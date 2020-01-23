package frc.team832.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.subsystems.Shooter;


import static frc.team832.robot.Robot.*;

public class SuperStructure extends SubsystemBase {

	private int stallCounter = 0;
	private StallState spindexerStallState = StallState.NOT_STALLED;
	private boolean hasStalled;
	private boolean hasHatch;

	private SuperstructureMode superstructureMode = SuperstructureMode.Idle, lastSuperstructureMode = SuperstructureMode.Idle;

	@Override
	public void periodic () {
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

	private void intake () {
		intake.intake(Constants.IntakeValues.IntakePowertrain.calculateMotorRpmFromSurfaceSpeed(10));
		spindexer.setClockwiseRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(60));
		pneumatics.extendIntake();
	}

	private void outtake () {
		intake.outtake(Constants.IntakeValues.IntakePowertrain.calculateMotorRpmFromSurfaceSpeed(5));
		spindexer.setCounterclockwiseRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(60));
		pneumatics.extendIntake();
	}

	private void prepareShoot () {
		shooter.setMode(Shooter.ShootMode.SpinUp);
		spindexer.stopSpin();
		shooter.setMode(Shooter.ShootMode.SpinUp);
		spindexer.setFeedRPM(Constants.SpindexerValues.FEED_RPM);
		pneumatics.propUp();
	}

	private void shooting () {
		spindexer.setCounterclockwiseRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(120));
	}

	private void idle () {
		stopIntake();
		pneumatics.retractProp();
		idleSpindexer();
	}

	public void stopIntake () {
		intake.stop();
		pneumatics.retractIntake();
	}

	public void idleSpindexer () {
		spindexer.setCounterclockwiseRPM(Constants.SpindexerValues.SpinPowertrain.calculateMotorRpmFromWheelRpm(30));
		spindexer.stopFeed();
	}

	public void setMode (SuperstructureMode mode) {
		if (superstructureMode != SuperstructureMode.Shooting || superstructureMode != SuperstructureMode.PrepareShoot) {
			lastSuperstructureMode = this.superstructureMode;
			this.superstructureMode = mode;
		}
	}

	public boolean isShooterPrepared () {
		return shooter.atShootingRpm() && spindexer.atFeedRpm();
	}

	public enum SuperstructureMode {
		Intake,
		Outtake,
		PrepareShoot,
		Shooting,
		Idle
	}

	public StallState isStalling(int PDPSlot, double stallCurrent, double stallSec) {
		int slowdownMultiplier = 8;
		int  stallLoops = (int)(stallSec * 20);
		stallLoops *= slowdownMultiplier;
		StallState stallState = StallState.NOT_STALLED;
		double motorCurrent = pdp.getChannelCurrent(PDPSlot);

		SmartDashboard.putNumber("Stall Count", stallCounter);
		SmartDashboard.putNumber("Stall Loops", stallLoops);
		if (motorCurrent >= stallCurrent) {
			stallCounter += slowdownMultiplier;
		} else if (motorCurrent < stallCurrent) {
			stallCounter--;
		}

		stallCounter = OscarMath.clip(stallCounter, 0, stallLoops + 1);
		if (stallCounter >= stallLoops) {
			hasStalled = true;
			stallState = StallState.STALLED;
		}
		else if (stallCounter == 0) {
			hasStalled = false;
			stallState = StallState.NOT_STALLED;
		}
		else if (hasStalled & stallCounter < stallLoops / 4) {
			stallState = StallState.LEAVING_STALL;
		}
		return stallState;
	}

	public void resetStall(){
		stallCounter = 0;
		spindexerStallState = StallState.NOT_STALLED;
	}

	public enum StallState {
		STALLED,
		LEAVING_STALL,
		NOT_STALLED
	}

	public void unStallSpindexer() {
		if (spindexer.isStalled()) spindexer.spinClockwise(0.3);
	}
}