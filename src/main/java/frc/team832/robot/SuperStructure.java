package frc.team832.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.robot.subsystems.Intake;
import frc.team832.robot.subsystems.Shooter;
import frc.team832.robot.subsystems.Spindexer;

public class SuperStructure extends SubsystemBase {
	public Intake intake;
	public Spindexer spindexer;
	public Shooter shooter;

	private ShootingMode shooterMode = ShootingMode.Idle, lastShooterMode = ShootingMode.Idle;
	private SuperstructureMode superstructureMode = SuperstructureMode.Idle, lastSuperstructureMode = SuperstructureMode.Idle;

	public SuperStructure(Intake intake, Spindexer spindexer, Shooter shooter) {
		this.intake = intake;
		this.spindexer = spindexer;
		this.shooter = shooter;
	}

	@Override
	public void periodic () {

	}

	public void runSuperStructure() {
		switch (superstructureMode) {
			case Intake:
				break;
			case Shoot:
				break;
			case Spin:
				break;
			case Idle:
				break;
		}
	}

	private void intake() {
		intake.intake(Constants.IntakeValues.intakePowertrain.calculateRpmFromSurfaceSpeed(10));
		spindexer.setClockwiseRPM(Constants.SpindexerValues.spinPowertrain.calculateMotorRpmFromWheelRpm(60));
	}

	private void outtake() {
		intake.outtake(Constants.IntakeValues.intakePowertrain.calculateRpmFromSurfaceSpeed(5));
		spindexer.setCounterclockwiseRPM(Constants.SpindexerValues.spinPowertrain.calculateMotorRpmFromWheelRpm(60));
	}

	public void stopIntake() {
		intake.stop();
		spindexer.stopSpin();
	}

	public void setShootingMode (ShootingMode mode) {
		lastShooterMode = this.shooterMode;
		this.shooterMode = mode;
	}

	public void setSuperstructureMode (SuperstructureMode mode) {
		lastSuperstructureMode = this.superstructureMode;
		this.superstructureMode = mode;
	}

	public enum ShootingMode {
		Preparing,
		Shooting,
		Idle
	}

	public enum SuperstructureMode {
		Intake,
		Spin,
		Shoot,
		Idle
	}

}
