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
	private SuperStructureMode currentMode = SuperStructureMode.IDLE;

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
	}

	public void runSuperStructureMode(){
        switch (currentMode) {
            case IDLE:

                break;
            case INTAKING:

                break;
            case SPINUP:

                break;
            case SHOOTING:

                break;
            case SPINDOWN:

                break;
        }

    }

	public void intake() {
		if (spindexer.isStalled()) {
			spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(60), Spindexer.SpinnerDirection.Clockwise);
		} else {
			spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(60), spindexer.getSpinnerDirection() == Spindexer.SpinnerDirection.Clockwise ? Spindexer.SpinnerDirection.CounterClockwise : Spindexer.SpinnerDirection.Clockwise);
		}
		intake.intake(Constants.IntakeValues.IntakePowertrain.calculateMotorRpmFromSurfaceSpeed(10));
		pneumatics.extendIntake();
	}

	public void outtake() {
		intake.outtake(Constants.IntakeValues.IntakePowertrain.calculateMotorRpmFromSurfaceSpeed(5));
		spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(60), Spindexer.SpinnerDirection.CounterClockwise);
		pneumatics.extendIntake();
	}

	public void prepareShoot() {
		spindexer.stopSpin();
		shooter.spinUp();
		pneumatics.propUp();
		shooter.setFeedRPM(Constants.SpindexerValues.FEED_RPM);
	}

	public void shoot() {
		shooter.setMode(Shooter.ShootMode.Shooting);
		spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(120), Spindexer.SpinnerDirection.CounterClockwise);
	}

	public void idleIntake() {
		intake.stop();
		pneumatics.retractIntake();
	}

	public void idleSpindexer() {
		spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(30), Spindexer.SpinnerDirection.CounterClockwise);
	}

	public void idleShooter(boolean interrupted) {
		shooter.idle();
		pneumatics.retractProp();
		idleSpindexer();
	}

	public void spindexerAntiStall() {
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
	    IDLE,
        INTAKING,
        SPINUP,
        SHOOTING,
        SPINDOWN
    }
}
