package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.robot.Constants;

public class SuperStructure extends SubsystemBase implements DashboardUpdatable {

	private Intake intake;
	private Shooter shooter;
	private Spindexer spindexer;
	private Pneumatics pneumatics;
	private SuperStructureMode currentMode = SuperStructureMode.IDLEALL, lastMode = SuperStructureMode.IDLEALL;

	private NetworkTableEntry dashboard_mode, dashboard_lastMode;

	public SuperStructure(Intake intake, Shooter shooter, Spindexer spindexer, Pneumatics pneumatics) {
		this.intake = intake;
		this.shooter = shooter;
		this.spindexer = spindexer;
		this.pneumatics = pneumatics;

        DashboardManager.addTab(this, this);

		dashboard_mode = DashboardManager.addTabItem(this, "Mode", "Default");
		dashboard_lastMode = DashboardManager.addTabItem(this, "Last Mode", "Default");
	}

	@Override
	public void periodic() {
		spindexerAntiStall();
		if (spindexer.isFull()) spindexer.setTargetPosition(spindexer.getNearestSafeRotationRelativeToFeeder());
	}

	private void updateSuperStructure(){
        switch (currentMode) {
			case IDLEALL:
				idle(true);
                break;
			case IDLELAST:
				idle(false);
				break;
            case INTAKING:
				intake();
                break;
			case OUTTAKING:
				outtake();
				break;
            case SPINUP:
				prepareShoot();
                break;
            case SHOOTING:
				shoot();
                break;
        }
    }

    public void setMode(SuperStructureMode mode) {
		if (currentMode != mode) {
			lastMode = currentMode;
			currentMode = mode;
			updateSuperStructure();
		}
	}

	private void intake() {
		intake.intake(Constants.IntakeValues.IntakePowertrain.calculateMotorRpmFromSurfaceSpeed(10));
		spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(60), Spindexer.SpinnerDirection.Clockwise);
		pneumatics.extendIntake();
	}

	private void outtake() {
		intake.outtake(Constants.IntakeValues.IntakePowertrain.calculateMotorRpmFromSurfaceSpeed(5));
		spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(60), Spindexer.SpinnerDirection.CounterClockwise);
		pneumatics.extendIntake();
	}

	private void prepareShoot() {
		spindexer.setTargetPosition(spindexer.getNearestSafeRotationRelativeToFeeder());
		shooter.spinUp();
		pneumatics.propUp();
	}

	private void shoot() {
		shooter.setMode(Shooter.ShootMode.Shooting);
		spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(120), Spindexer.SpinnerDirection.CounterClockwise);
	}

	public void idle(boolean idleAll) {
		if (idleAll) {
			idleIntake();
			idleSpindexer();
			idleShooter();
		} else {
			handleIdle();
		}
	}

	private void idleIntake() {
		intake.stop();
		pneumatics.retractIntake();
	}

	private void idleSpindexer() {
		spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(30), Spindexer.SpinnerDirection.CounterClockwise);
	}

	private void idleShooter(){
		shooter.idle();
		pneumatics.retractProp();
	}

	public double getFeederRotationRelativeToSpindexer() {
		return -getSpindexerRotationRelativeToFeeder();
	}

	public double getSpindexerRotationRelativeToFeeder() {
		return spindexer.getRelativeRotations() - shooter.getTurretRotations();
	}

	public double calculateSpindexerPosRelativeToFeeder(double targetPos) {
		return targetPos - shooter.getTurretRotations();
	}

	private void spindexerAntiStall() {
//		if (spindexer.isStalled()) spindexer.switchSpin();
	}

	public boolean isSpindexerUnloaded() {
		return spindexer.isUnloaded();
	}

	public boolean isShooterPrepared() {
		return shooter.readyToShoot();
	}

    @Override
    public String getDashboardTabName() {
        return "Superstructure";
    }

    @Override
    public void updateDashboardData() {
		dashboard_mode.setString(getModeString());
		dashboard_lastMode.setString(getLastModeString());
    }

    public enum SuperStructureMode {
	    IDLEALL,
		IDLELAST,
        INTAKING,
		OUTTAKING,
        SPINUP,
        SHOOTING
    }

    private void handleIdle() {
		switch (lastMode) {
			case SPINUP:
				idleShooter();
				break;
			case SHOOTING:
				idleShooter();
				break;
			case INTAKING:
				idleIntake();
				idleSpindexer();
				break;
			case OUTTAKING:
				idleIntake();
				idleSpindexer();
				break;
			default:
				break;
		}
	}

	private String getModeString() {
		switch (currentMode) {
			case INTAKING:
				return "Intaking";
			case OUTTAKING:
				return "Outtaking";
			case IDLEALL:
				return "Idle All";
			case IDLELAST:
				return "Idle Last";
			default:
				return "AHHHHHH";
		}
	}

	private String getLastModeString() {
		switch (lastMode) {
			case INTAKING:
				return "Intaking";
			case OUTTAKING:
				return "Outtaking";
			case IDLEALL:
				return "Idle All";
			case IDLELAST:
				return "Idle Last";
			default:
				return "AHHHHHH";
		}
	}
}
