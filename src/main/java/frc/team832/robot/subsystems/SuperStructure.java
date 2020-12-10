package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.*;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;
import frc.team832.robot.utilities.positions.BallPosition;
import frc.team832.robot.utilities.state.ShooterCalculations;

public class SuperStructure extends SubsystemBase {

	private final Intake intake;
	private final Shooter shooter;
	private final Spindexer spindexer;
	private final Turret turret;
	private final Vision vision;

	private final NetworkTableEntry dashboard_hoodVolts, dashboard_hoodTargetVolts;

	public final IdleCommand idleCommand;
	public final TargetingCommand targetingCommand;
	public final ShootCommandGroup shootOnTarget;
	public final CloseRangeShootCommandGroup closeShoot;
	public final IntakeCommand intakeCommand;

	private double hoodVoltage = 2.72, spindexerRpm = 15;


	public SuperStructure(Intake intake, Shooter shooter, Spindexer spindexer, Turret turret, Vision vision) {
		this.intake = intake;
		this.shooter = shooter;
		this.spindexer = spindexer;
		this.turret = turret;
		this.vision = vision;

		idleCommand = new IdleCommand();
		targetingCommand = new TargetingCommand();
		shootOnTarget = new ShootCommandGroup();
		closeShoot = new CloseRangeShootCommandGroup();
		intakeCommand = new IntakeCommand();

		DashboardManager.addTab(this);
		dashboard_hoodTargetVolts = DashboardManager.addTabItem(this, "Target Volts", 0.0);
		dashboard_hoodVolts = DashboardManager.addTabItem(this, "Current Volts", 0.0);

		vision.driverMode(false);
	}

	@Override
	public void periodic() {
		dashboard_hoodTargetVolts.setDouble(hoodVoltage);
		dashboard_hoodVolts.setDouble(shooter.getPotentiometer());
	}

	public void intake(double power, double spinRPM, Spindexer.SpinnerDirection direction) {
		intake.intake(power);
		spindexer.setSpinRPM(spinRPM, direction);
		intake.extendIntake();
	}

	public void outtake(double power, double spinRPM, Spindexer.SpinnerDirection direction) {
		intake.outtake(power);
		spindexer.setSpinRPM(spinRPM, direction);
		intake.extendIntake();
	}

    public void trackTarget() {
	    if (vision.hasTarget()) {
            turret.trackTarget(spindexer.getVelocity());
            shooter.trackTarget();
        } else {
	    	turret.setTurretTargetDegrees(0.0, true);
		}
    }

    public void shootAtTarget() {
		if (vision.hasTarget()) {
			turret.trackTarget(spindexer.getVelocity());
			shooter.trackTarget();
			shooter.setFeedRPM(Constants.ShooterValues.FeedRpm);
		} else {
			turret.setTurretTargetDegrees(0.0, true);
		}
	}

    public void stopTrackTarget() {
        turret.setTurretTargetDegrees(turret.getDegrees(), false);
    }

	private void spindexerAntiStall() {
//		if (spindexer.isStalled()) spindexer.switchSpin();
	}

	public double getFeederRotationRelativeToSpindexer() {
		return -getSpindexerRotationRelativeToFeeder();
	}

	public double getSpindexerRotationRelativeToFeeder() {
		return spindexer.getRelativeRotations() - turret.getRotations();
	}

	public double calculateSpindexerRotRelativeToFeeder(double targetPos) {
		return targetPos - turret.getRotations();
	}

	public double getNearestSafeRotationRelativeToFeeder() {
		return getNearestBallRotationRelativeToFeeder() + calculateSpindexerRotRelativeToFeeder(0.1);
	}

	public double getNearestBallRotationRelativeToFeeder() {
		return calculateSpindexerRotRelativeToFeeder(spindexer.getNearestBallPosition().rotations);
	}

	public boolean isSpindexerReadyShoot(double safePos, double curPos) {
		return Math.abs(safePos - curPos) < 0.05;
	}

	public boolean isFeederOverBallSlot() {
		return Math.abs(spindexer.getRelativeRotations() - getNearestBallRotationRelativeToFeeder()) < 0.05;
	}

	public boolean isFeederOverBallPosition(BallPosition position) {
		return Math.abs(spindexer.getRelativeRotations() - position.rotations) < 0.05;
	}

	public boolean isSpindexerUnloaded() {
		return spindexer.isUnloaded();
	}

	public boolean isShooterPrepared() {
		return shooter.readyToShoot() && isSpindexerReadyShoot(getNearestSafeRotationRelativeToFeeder(), spindexer.getRelativeRotations());
	}

	public void dumbShoot(){
		shooter.dumbShoot();
		turret.setForward(false);
	}

	private class IdleCommand extends InstantCommand {
		IdleCommand() {
			addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
		}

		@Override
		public void initialize() {
			idleAll();
			turret.setTurretTargetDegrees(0, false);
		}
	}

	public class IntakeCommand extends CommandBase {
		IntakeCommand() {
			addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
		}

		@Override
		public void initialize() {
			shooter.setFeedRPM(0);
			turret.setIntake();
		}

		@Override
		public void execute() {
			if (Math.signum(spindexerRpm) == -1) {
				intake(0.7, -spindexerRpm, Spindexer.SpinnerDirection.CounterClockwise);
			} else {
				intake(0.7, spindexerRpm, Spindexer.SpinnerDirection.Clockwise);
			}
		}

		@Override
		public void end(boolean interrupted) {
			idleIntake();
		}
	}

	public class TargetingCommand extends CommandBase{
		TargetingCommand() {
			addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
		}

		@Override
		public void initialize() {
			spindexer.idle();
			turret.setForward(true);
		}

		@Override
		public void execute() {
			trackTarget();
//			spindexer.setTargetRotation(getNearestSafeRotationRelativeToFeeder());might be unnecessary
		}
	}

	public class ShootCommandGroup extends ParallelCommandGroup {
		public ShootCommandGroup() {
			addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
			addCommands(
					// tracking target
					new FunctionalCommand(() -> turret.setForward(true), SuperStructure.this::trackTarget, (interrupted) -> {}, () -> false),
					// wait then shoot
					new SequentialCommandGroup(
							new WaitCommand(0.5),
							new FunctionalCommand(
									() -> spindexer.setSpinRPM(ShooterCalculations.getSpindexerRpm(), Spindexer.SpinnerDirection.Clockwise),
									SuperStructure.this::shootAtTarget,
									(interrupted) -> { idleSpindexer(); idleShooter(); },
									() -> false
							)
					)
			);
		}
	}

	public class CloseRangeShootCommandGroup extends ParallelCommandGroup {
		public CloseRangeShootCommandGroup() {
			addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
			addCommands(
					new InstantCommand(SuperStructure.this::dumbShoot),
					// wait then shoot
					new SequentialCommandGroup(
							new WaitCommand(1),
							new FunctionalCommand(
									() -> spindexer.setSpinRPM(90, Spindexer.SpinnerDirection.Clockwise),
									() -> shooter.setFeedRPM(3000),
									(interrupted) -> { idleSpindexer(); idleShooter(); },
									() -> false
							)
					)
			);
		}
	}

	public void configureSpindexerRPMSlider(double spindexerSlider) {
		spindexerRpm = OscarMath.clipMap(spindexerSlider, -1 , 1, -15, 15);
	}

	public void setSpindexerIntakeRpmDefault() {
		spindexerRpm = 10;
	}

	public void idleIntake() {
		intake.stop();
		intake.retractIntake();
	}

	public void idleSpindexer() {
		spindexer.idle();
	}

	public void idleShooter() {
		shooter.idle();
	}

	public void idleAll() {
		idleShooter();
		idleIntake();
		idleSpindexer();
	}
}
