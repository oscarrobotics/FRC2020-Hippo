package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.robot.Constants;
public class SuperStructure extends SubsystemBase {

	private Intake intake;
	private Shooter shooter;
	private Spindexer spindexer;
	private Pneumatics pneumatics;

	public SuperStructure(Intake intake, Shooter shooter, Spindexer spindexer, Pneumatics pneumatics) {
		this.intake = intake;
		this.shooter = shooter;
		this.spindexer = spindexer;
		this.pneumatics = pneumatics;
	}

	@Override
	public void periodic() {
		spindexerAntiStall();
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
		spindexer.setFeedRPM(Constants.SpindexerValues.FEED_RPM);
	}

	public void shoot() {
		shooter.setMode(Shooter.ShootMode.Shooting);
		spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(120), Spindexer.SpinnerDirection.CounterClockwise);
	}

	public void idleIntake() {
		stopIntake();
	}

	public void idleSpindexer() {
		spindexer.setSpinRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(30), Spindexer.SpinnerDirection.CounterClockwise);
	}

	public void idleShooter(boolean interrupted) {
		shooter.idle();
		pneumatics.retractProp();
	}

	public void stopIntake() {
		intake.stop();
		pneumatics.retractIntake();
	}

	public void spindexerAntiStall() {
		if (spindexer.isStalled()) spindexer.switchSpin();
	}

	public boolean isSpindexerUnloaded() {
		return spindexer.isUnloaded();
	}

	public boolean isShooterPrepared() {
		return shooter.readyToShoot() && spindexer.atFeedRpm();
	}
}