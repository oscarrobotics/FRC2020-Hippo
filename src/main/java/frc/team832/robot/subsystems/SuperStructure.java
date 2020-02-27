package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;
import frc.team832.robot.utilities.positions.BallPosition;
import frc.team832.robot.utilities.state.SpindexerStatus;

public class SuperStructure extends SubsystemBase implements DashboardUpdatable {

	private Intake intake;
	private Shooter shooter;
	private Spindexer spindexer;

	private NetworkTableEntry dashboard_mode, dashboard_lastMode;
	private SuperstructureState _state;

	private PrepareShootCommand prepareShootCommand = new PrepareShootCommand();
	private ShootCommand shootCommand = new ShootCommand();

	public SuperStructure(Intake intake, Shooter shooter, Spindexer spindexer) {
		this.intake = intake;
		this.shooter = shooter;
		this.spindexer = spindexer;

        DashboardManager.addTab(this, this);

		dashboard_mode = DashboardManager.addTabItem(this, "Mode", "Default");
		dashboard_lastMode = DashboardManager.addTabItem(this, "Last Mode", "Default");
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
		return spindexer.getRelativeRotations() - shooter.getTurretRotations();
	}

	public double calculateSpindexerRotRelativeToFeeder(double targetPos) {
		return targetPos - shooter.getTurretRotations();
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

	public void trackTarget() {
		shooter.trackTarget();
	}

	@Override
    public String getDashboardTabName() {
        return "Superstructure";
    }

    @Override
    public void updateDashboardData() {

    }

    private class IntakeCommand extends CommandBase {

	}

    private class PrepareShootCommand extends SequentialCommandGroup {
		public PrepareShootCommand() {
			addCommands(
					new FunctionalCommand(
							SuperStructure.this::moveSpindexerToSafePos,
							() -> {},
							(ignored) -> idleSpindexer(),
							() -> isSpindexerReadyShoot(getNearestSafeRotationRelativeToFeeder(), spindexer.getRelativeRotations())
					)
			);
		}
	}

    private class ShootCommand extends CommandBase {
		public ShootCommand() {
			addRequirements(shooter, intake, spindexer);
		}

		@Override
		public void initialize() {
			shooter.setMode(Shooter.ShootMode.Shooting);
		}

		@Override
		public void execute() {
		}

		@Override
		public void end(boolean interrupted) {

		}

		@Override
		public boolean isFinished() {
			return spindexer.getState().equals(SpindexerStatus.SpindexerState.EMPTY);
		}
	}

    public void trackTarget() {
        isVision = true;
        vision.driverMode(false);
        hoodTrackTarget();
    }

    public void stopTrackTarget() {
        isVision = false;
        vision.driverMode(true);
    }

    public void setState(SuperstructureState state) {
		_state = state;
	}

	private void handleState() {
		switch (_state) {
			case IDLE:
				idleAll();
				break;
			case INTAKE:
				intake();

				break;
			case TARGETING:
				break;
			case SHOOTING:
				break;
		}
	}

    public enum SuperstructureState {
		IDLE,
		INTAKE,
		TARGETING,
		SHOOTING
	}
}
