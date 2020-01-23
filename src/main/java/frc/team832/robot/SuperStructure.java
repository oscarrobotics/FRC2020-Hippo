package frc.team832.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.robot.subsystems.Shooter;


import static frc.team832.robot.Robot.*;

public class SuperStructure extends SubsystemBase {

	private SuperstructureMode superstructureMode = SuperstructureMode.Idle, lastSuperstructureMode = SuperstructureMode.Idle;

	@Override
	public void periodic () {
		runSuperStructure();
	}

	public void runSuperStructure() {
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
		intake.intake(Constants.IntakeValues.IntakePowertrain.calculateRpmFromSurfaceSpeed(10));
		spindexer.setClockwiseRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(60));
		pneumatics.extendIntake();
	}

	private void outtake() {
		intake.outtake(Constants.IntakeValues.IntakePowertrain.calculateRpmFromSurfaceSpeed(5));
		spindexer.setCounterclockwiseRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(60));
		pneumatics.extendIntake();
	}

	private void prepareShoot() {
		shooter.setMode(Shooter.ShootMode.SpinUp);
		spindexer.stopSpin();
		shooter.setRPM(Constants.ShooterValues.SHOOTING_RPM);
		spindexer.setFeedRPM(Constants.SpindexerValues.FEED_RPM);
		pneumatics.propUp();
	}

	private void shooting() {
		spindexer.setCounterclockwiseRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(120));
	}

	private void idle() {
		stopIntake();
		pneumatics.retractProp();
		spindexer.setCounterclockwiseRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(30));
		spindexer.stopFeed();
		wheelOfFortune.stopSpin();
	}

	public void stopIntake() {
		intake.stop();
		pneumatics.retractIntake();
	}

	public void setMode(SuperstructureMode mode) {
		if (superstructureMode != SuperstructureMode.Shooting || superstructureMode != SuperstructureMode.PrepareShoot) {
			lastSuperstructureMode = this.superstructureMode;
			this.superstructureMode = mode;
		}
	}

	public boolean isShooterPrepared() {
		return shooter.atShootingRpm() && spindexer.atFeedRpm();
	}

	public enum SuperstructureMode {
		Intake,
		Outtake,
		PrepareShoot,
		Shooting,
		Idle
	}

}
