package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverinput.controllers.StratComInterface;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;
import frc.team832.robot.OI;

public class SuperStructure extends SubsystemBase implements DashboardUpdatable {

	private Intake intake;
	private Shooter shooter;
	private Spindexer spindexer;
	private Pneumatics pneumatics;

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

	public void intake() {
		intake.intake(Constants.IntakeValues.IntakePowertrain.calculateMotorRpmFromSurfaceSpeed(10));
		spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(60), Spindexer.SpinnerDirection.Clockwise);
	}

	public void outtake() {
		intake.outtake(Constants.IntakeValues.IntakePowertrain.calculateMotorRpmFromSurfaceSpeed(5));
		spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(60), Spindexer.SpinnerDirection.CounterClockwise);
	}

	public void dumbIntake(double power) {
		intake.intake(OscarMath.map(power, -1, 1, 0, 1));
		pneumatics.extendIntake();
	}

	public void dumbIntakeIdle() {
		intake.stop();
		pneumatics.retractIntake();
	}

	public void prepareShoot() {
		spindexer.setTargetPosition(spindexer.getNearestSafeRotationRelativeToFeeder());
		shooter.spinUp();
		pneumatics.propUp();
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

	public void idleShooter(){
		shooter.idle();
		pneumatics.retractProp();
	}

	public void idleAll() {
		idleShooter();
		idleIntake();
		idleSpindexer();
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

    }

}
