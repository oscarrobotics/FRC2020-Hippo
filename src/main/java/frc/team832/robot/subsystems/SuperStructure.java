package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.robot.Constants;

public class SuperStructure extends SubsystemBase {

	private SuperstructureMode superstructureMode = SuperstructureMode.Idle, lastSuperstructureMode = SuperstructureMode.Idle;

	private Intake intake;
	private Shooter shooter;
	private Spindexer spindexer;
	private Pneumatics pneumatics;

	public SuperStructure(Intake intake, Shooter shooter, Spindexer spindexer, Pneumatics pneumatics){
		this.intake = intake;
		this.shooter = shooter;
		this.spindexer = spindexer;
		this.pneumatics = pneumatics;
	}

	@Override
	public void periodic() {
		runSuperStructure();
	}

	public void runSuperStructure () {
		switch (superstructureMode) {
			case Intake:
				intake();
				break;
			case Outtake:
				outtake();
				break;
			case PrepareShoot:
				prepareShoot();
				break;
			case Shooting:
				shooting();
				break;
			case Idle:
				idle();
				break;
		}
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
		spindexer.setFeedRPM(Constants.SpindexerValues.FEED_RPM);
	}

	private void shooting() {
		shooter.setMode(Shooter.ShootMode.Shooting);
		spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(120), Spindexer.SpinnerDirection.CounterClockwise);
	}

	private void idle() {
		stopIntake();
		pneumatics.retractProp();
		idleSpindexer();
	}

	public void stopIntake() {
		intake.stop();
		pneumatics.retractIntake();
	}

	public void idleSpindexer() {
		if (spindexer.isStalled()) {
			spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(30), Spindexer.SpinnerDirection.CounterClockwise);
		} else {
			spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(30), spindexer.getSpinnerDirection() == Spindexer.SpinnerDirection.Clockwise ? Spindexer.SpinnerDirection.CounterClockwise : Spindexer.SpinnerDirection.Clockwise);
		}
		spindexer.stopFeed();
	}

	public void setMode(SuperstructureMode mode) {
		if (superstructureMode != SuperstructureMode.Shooting && superstructureMode != SuperstructureMode.PrepareShoot) {
			lastSuperstructureMode = this.superstructureMode;
			this.superstructureMode = mode;
		}
	}

	public boolean isShooterPrepared() {
		return shooter.readyToShoot() && spindexer.atFeedRpm();
	}

	public enum SuperstructureMode {
		Intake,
		Outtake,
		PrepareShoot,
		Shooting,
		Idle
	}
}