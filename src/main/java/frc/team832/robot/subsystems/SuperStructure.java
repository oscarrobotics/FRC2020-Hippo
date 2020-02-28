package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.*;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;
import frc.team832.robot.utilities.positions.BallPosition;
import frc.team832.robot.utilities.state.ShooterCalculations;

public class SuperStructure extends SubsystemBase implements DashboardUpdatable {

	private final Intake intake;
	private final Shooter shooter;
	private final Spindexer spindexer;
	private final Turret turret;
	private final Vision vision;

	private final NetworkTableEntry dashboard_mode, dashboard_hoodVolts, dashboard_hoodTargetVolts;
	private SuperstructureState _state = SuperstructureState.IDLE, _lastState = SuperstructureState.IDLE;
	private ShootingState _shootingState = ShootingState.PREPARE;

	public final IdleCommand idleCommand;
	public final TargetingCommand targetingCommand;
	public final ShootCommand shootCommand;
	public final IntakeCommand intakeCommand;

	private double flywheelRpm  = 5000, hoodVoltage = 2.72;


	public SuperStructure(Intake intake, Shooter shooter, Spindexer spindexer, Turret turret, Vision vision) {
		this.intake = intake;
		this.shooter = shooter;
		this.spindexer = spindexer;
		this.turret = turret;
		this.vision = vision;

		idleCommand = new IdleCommand();
		targetingCommand = new TargetingCommand();
		shootCommand = new ShootCommand();
		intakeCommand = new IntakeCommand();

		DashboardManager.addTab(this, this);
		dashboard_mode = DashboardManager.addTabItem(this, "State", SuperstructureState.INVALID.toString());
		dashboard_hoodTargetVolts = DashboardManager.addTabItem(this, "Target Volts", 0.0);
		dashboard_hoodVolts = DashboardManager.addTabItem(this, "Current Volts", 0.0);

		vision.driverMode(false);
	}

	@Override
	public void periodic() {
		handleState();
	}

	public void intake() {
		intake.setIntakeRPM(Constants.IntakeValues.IntakePowertrain.calculateMotorRpmFromSurfaceSpeed(10));
		spindexer.setSpinRPM(60, Spindexer.SpinnerDirection.Clockwise);
		intake.extendIntake();
	}

	public void outtake() {
		intake.setOuttakeRPM(Constants.IntakeValues.IntakePowertrain.calculateMotorRpmFromSurfaceSpeed(5));
		spindexer.setSpinRPM(60, Spindexer.SpinnerDirection.CounterClockwise);
		intake.extendIntake();
	}

	public void dumbIntake(double slider) {
		intake.setIntakeRPM(OscarMath.map(slider, -1, 1, 0, Motor.kNEO550.freeSpeed));
		intake.extendIntake();
	}

	public void dumbIntakeIdle() {
		intake.stop();
		intake.retractIntake();
	}

	public void prepareShoot() {
		spindexer.setTargetRotation(getNearestSafeRotationRelativeToFeeder());
		shooter.setMode(Shooter.ShootMode.SpinUp);
//		Drivetrain.propUp();
	}

	public void shoot() {
		shooter.setMode(Shooter.ShootMode.Shooting);
		spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(60), Spindexer.SpinnerDirection.CounterClockwise);
	}

	public void idleIntake() {
		intake.stop();
		intake.retractIntake();
	}

	public void idleSpindexer() {
		spindexer.stopSpin();
//		spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(30), Spindexer.SpinnerDirection.CounterClockwise);
	}

	public void idleShooter() {
		shooter.idle();
//		Drivetrain.retractProp();
	}

	public void idleAll() {
		idleShooter();
		idleIntake();
		idleSpindexer();
	}

    public void trackTarget() {
		turret.setVisionMode(true);
	    if (vision.getTarget().isValid) {
            turret.setTurretTargetDegrees(ShooterCalculations.visionYaw + turret.getDegrees(), true);
        } else {
	    	turret.setTurretTargetDegrees(0.0, true);
		}

    }

    public void stopTrackTarget() {
        turret.setTurretTargetDegrees(turret.getDegrees(), false);
    }

	public void moveSpindexerToSafePos() {
		spindexer.setTargetRotation(getNearestSafeRotationRelativeToFeeder());
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

	public boolean isOverBallSlot() {
		return Math.abs(spindexer.getRelativeRotations() - getNearestBallRotationRelativeToFeeder()) < 0.05;
	}

	public boolean isOverBallPosition(BallPosition position) {
		return Math.abs(spindexer.getRelativeRotations() - position.rotations) < 0.05;
	}

	public boolean isSpindexerUnloaded() {
		return spindexer.isUnloaded();
	}

	public boolean isShooterPrepared() {
		return shooter.readyToShoot() && isSpindexerReadyShoot(getNearestSafeRotationRelativeToFeeder(), spindexer.getRelativeRotations());
	}

	@Override
    public String getDashboardTabName() {
        return "Superstructure";
    }

    @Override
    public void updateDashboardData() {
		dashboard_mode.setString(_state.toString());
		dashboard_hoodTargetVolts.setDouble(hoodVoltage);
		dashboard_hoodVolts.setDouble(shooter.getPotentiometer());
    }

	private class IdleCommand extends InstantCommand {
		IdleCommand() {
			addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
		}

		@Override
		public void initialize() {
			idleAll();
			turret.setVisionMode(false);
			_shootingState = ShootingState.PREPARE;
			if(_lastState == SuperstructureState.INTAKE){
				turret.setIntake();
			} else {
				turret.setTurretTargetDegrees(0, false);
			}
//			turret.setTurretTargetDegrees(0, false);
		}
	}

    private class IntakeCommand extends InstantCommand {
		IntakeCommand() {
			addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
		}

		@Override
		public void initialize() {
			shooter.setDumbRPM(0);
			shooter.setFeedRPM(0);
			spindexer.setSpinRPM(25, Spindexer.SpinnerDirection.Clockwise);
			intake.extendIntake();
			intake.intake(1.0);
			vision.driverMode(true);
			turret.setIntake();
		}

		@Override
		public void end(boolean interrupted) {
		}
	}

    private class TargetingCommand extends SequentialCommandGroup {
		TargetingCommand() {
			addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
		}

		@Override
		public void initialize() {
			vision.driverMode(false);
			shooter.idle();
			spindexer.idle();
			turret.setForward(true);
		}

		@Override
		public void execute() {
			trackTarget();
		}
	}

    private class ShootCommand extends CommandBase {
		ShootCommand() {
			addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
		}

		@Override
		public void initialize() {
			shooter.setMode(Shooter.ShootMode.Shooting);
			shooter.setDumbRPM(5000);
			shooter.setFeedRPM(3000);
			turret.setForward(true);
			vision.driverMode(false);
		}

		@Override
		public void execute() {
			trackTarget();
			shooter.setHood(hoodVoltage);
			shooter.setDumbRPM(flywheelRpm);
			if (_shootingState == ShootingState.FIRING) {
				spindexer.setSpinRPM(70, Spindexer.SpinnerDirection.Clockwise);
			} else {
				spindexer.setSpinRPM(0, Spindexer.SpinnerDirection.Clockwise);
			}
		}
	}

	public void setShooterParametersSlider(double rpmSlider, double hoodSlider) {
		flywheelRpm = OscarMath.clipMap(rpmSlider, -1, 1, 0, 5000);
		hoodVoltage = OscarMath.clipMap(hoodSlider, -1, 1, Constants.ShooterValues.HoodMax, Constants.ShooterValues.HoodMin);
	}

	public void setShooterParametersdefault() {
		flywheelRpm = 5000;
		hoodVoltage = 2.72;
	}

    public void setState(SuperstructureState state) {
		_lastState = _state;
		_state = state;
	}

	public void setShootingState(ShootingState state){
		_shootingState = state;
	}

	private void handleState() {
		switch (_state) {
			case IDLE:
				idleCommand.schedule();
				break;
			case INTAKE:
				intakeCommand.schedule();
				break;
			case TARGETING:
				targetingCommand.schedule();
				break;
			case SHOOTING:
				shootCommand.schedule();
				break;
		}
	}

    public enum SuperstructureState {
		INVALID,
		IDLE,
		INTAKE,
		TARGETING,
		SHOOTING
	}

	public enum ShootingState {
		PREPARE,
		FIRING
	}
}
