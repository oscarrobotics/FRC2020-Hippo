package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;
import frc.team832.robot.utilities.positions.BallPosition;

public class SuperStructure extends SubsystemBase implements DashboardUpdatable {

	private Intake intake;
	private Shooter shooter;
	private Spindexer spindexer;

	private NetworkTableEntry dashboard_mode, dashboard_lastMode;

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
		spindexer.updateStatus(isOverBallSlot());
		spindexerAntiStall();
		if (spindexer.isFull()) spindexer.setTargetRotation(getNearestSafeRotationRelativeToFeeder());
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
		shooter.spinUp();
//		Drivetrain.propUp();
	}

	public void shoot() {
		shooter.setMode(Shooter.ShootMode.Shooting);
		spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(120), Spindexer.SpinnerDirection.CounterClockwise);
	}

	public void idleIntake() {
		intake.stop();
		intake.retractIntake();
	}

	public void idleSpindexer() {
		spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(30), Spindexer.SpinnerDirection.CounterClockwise);
	}

	public void idleShooter(){
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



	@Override
    public String getDashboardTabName() {
        return "Superstructure";
    }

    @Override
    public void updateDashboardData() {

    }

}
