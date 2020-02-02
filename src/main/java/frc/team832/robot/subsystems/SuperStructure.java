package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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

	private NetworkTableEntry mode;

	public SuperStructure(Intake intake, Shooter shooter, Spindexer spindexer, Pneumatics pneumatics) {
		this.intake = intake;
		this.shooter = shooter;
		this.spindexer = spindexer;
		this.pneumatics = pneumatics;

        DashboardManager.addTab(this);

//        DashboardManager.addTabItem(this, "Mode", )
    }

	@Override
	public void periodic() {
		spindexerAntiStall();
		if (currentMode != lastMode) {
			updateSuperStructure();
		}
		lastMode = currentMode;
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
            case SPINUP:
				prepareShoot();
                break;
            case SHOOTING:
				shoot();
                break;
        }
    }

    public void setMode(SuperStructureMode mode) {
		this.currentMode = mode;
	}

	private void intake() {
		if (spindexer.isStalled()) {
			spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(60), Spindexer.SpinnerDirection.Clockwise);
		} else {
			spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(60), spindexer.getSpinnerDirection() == Spindexer.SpinnerDirection.Clockwise ? Spindexer.SpinnerDirection.CounterClockwise : Spindexer.SpinnerDirection.Clockwise);
		}
		intake.intake(Constants.IntakeValues.IntakePowertrain.calculateMotorRpmFromSurfaceSpeed(10));
		pneumatics.extendIntake();
	}

	private void outtake() {
		intake.outtake(Constants.IntakeValues.IntakePowertrain.calculateMotorRpmFromSurfaceSpeed(5));
		spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(60), Spindexer.SpinnerDirection.CounterClockwise);
		pneumatics.extendIntake();
	}

	private void prepareShoot() {
		spindexer.stopSpin();
		shooter.spinUp();
		pneumatics.propUp();
		shooter.setFeedRPM(Constants.SpindexerValues.FEED_RPM);
	}

	private void shoot() {
		shooter.setMode(Shooter.ShootMode.Shooting);
		spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(120), Spindexer.SpinnerDirection.CounterClockwise);
	}

	public void idle(boolean idleAll) {
		if (idleAll) {
			setMode(SuperStructureMode.IDLEALL);
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
		idleSpindexer();
	}

	private void spindexerAntiStall() {
		if (spindexer.isStalled()) spindexer.switchSpin();
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

    }

    public enum SuperStructureMode {
	    IDLEALL,
		IDLELAST,
        INTAKING,
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
				break;
			default:
				break;
		}
		setMode(SuperStructureMode.IDLELAST);
	}
}
