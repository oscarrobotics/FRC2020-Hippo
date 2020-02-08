package frc.team832.robot.accesories;

import frc.team832.lib.drive.SmartDiffDrive;
import frc.team832.lib.driverinput.oi.DriveAxesSupplier;
import frc.team832.lib.driverinput.oi.SticksDriverOI;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;

import static frc.team832.robot.Robot.oi;

public class TankDriveProfile {
	public final boolean useFF, useCloseLoop;
	public SmartDiffDrive.LoopMode loopMode;
	public double leftPower, rightPower;

	public TankDriveProfile (boolean useFF, boolean useCloseLoop) {
		this.useFF = useFF;
		this.useCloseLoop = useCloseLoop;
		if (useCloseLoop) loopMode = SmartDiffDrive.LoopMode.VELOCITY;
		else loopMode = SmartDiffDrive.LoopMode.PERCENTAGE;
	}

	public void calculateTankSpeeds() {
		boolean isPreciseRotate = ((SticksDriverOI) oi.driverOI).rightStick.two.get();
		boolean isRotate =  ((SticksDriverOI) oi.driverOI).rightStick.trigger.get();
		boolean driveStraight = ((SticksDriverOI) oi.driverOI).leftStick.trigger.get();

		if (driveStraight) {
			StoreDriveSpeeds straight = getTankStraightProfile();
			if (isPreciseRotate || isRotate) {
				StoreDriveSpeeds rotate;
				if (isPreciseRotate) {
					rotate = getTankPreciseRotateProfile();
				} else {
					rotate = getTankRotateProfile();
				}
				setSpeeds(straight.leftPow + rotate.leftPow, straight.rightPow + rotate.rightPow);
			} else {
				setSpeeds(straight.leftPow, straight.rightPow);
			}
		} else {
			if (isPreciseRotate) {
				StoreDriveSpeeds rotateOnCenter = getTankRotateOnCenterProfile();
				setSpeeds(rotateOnCenter.leftPow, rotateOnCenter.rightPow);
			} else {
				StoreDriveSpeeds profile = getTankNormalProfile();
				setSpeeds(profile.leftPow, profile.rightPow);
			}
		}
	}

	public StoreDriveSpeeds getTankStraightProfile() {
		DriveAxesSupplier axes = oi.driverOI.getGreenbergDriveAxes();
		double power;

		if (useFF) {
			power = OscarMath.signumPow(getPower(axes.getLeftY(), 0.1) * Constants.DrivetrainValues.StickDriveMultiplier, 2);
		} else {
			power = OscarMath.signumPow(axes.getLeftY() * Constants.DrivetrainValues.StickDriveMultiplier, 2);
		}

		return new StoreDriveSpeeds(power, power);
	}

	public StoreDriveSpeeds getTankNormalProfile() {
		DriveAxesSupplier axes = oi.driverOI.getGreenbergDriveAxes();
		double rightPow, leftPow;

		if (useFF) {
			rightPow = OscarMath.signumPow(getPower(axes.getRightY(), 0.1) * Constants.DrivetrainValues.StickDriveMultiplier, 2);
			leftPow = OscarMath.signumPow(getPower(axes.getLeftY(), 0.1) * Constants.DrivetrainValues.StickDriveMultiplier, 2);
		} else {
			rightPow = OscarMath.signumPow(axes.getRightY() * Constants.DrivetrainValues.StickDriveMultiplier, 2);
			leftPow = OscarMath.signumPow(axes.getLeftY() * Constants.DrivetrainValues.StickDriveMultiplier, 2);
		}

		return new StoreDriveSpeeds(leftPow, rightPow);
	}

	public StoreDriveSpeeds getTankRotateProfile() {
		DriveAxesSupplier axes = oi.driverOI.getGreenbergDriveAxes();
		double rightPow, leftPow;

		if (useFF) {
			rightPow = -OscarMath.signumPow(getPower(axes.getRightX(), 0.1) * Constants.DrivetrainValues.StickRotateMultiplier, 1.5);
			leftPow = OscarMath.signumPow(getPower(axes.getRightX(), 0.1) * Constants.DrivetrainValues.StickRotateMultiplier, 1.5);
		} else {
			rightPow = -OscarMath.signumPow(axes.getRightX() * Constants.DrivetrainValues.StickRotateMultiplier, 1.5);
			leftPow = OscarMath.signumPow(axes.getRightX() * Constants.DrivetrainValues.StickRotateMultiplier, 1.5);
		}

		return new StoreDriveSpeeds(leftPow, rightPow);
	}

	public StoreDriveSpeeds getTankPreciseRotateProfile() {
		DriveAxesSupplier axes = oi.driverOI.getGreenbergDriveAxes();
		double rightPow, leftPow;

		if (useFF) {
			rightPow = -OscarMath.signumPow(getPower(axes.getRotation(), 0.1) * Constants.DrivetrainValues.StickRotateMultiplier, 2);
			leftPow = OscarMath.signumPow(getPower(axes.getRotation(), 0.1) * Constants.DrivetrainValues.StickRotateMultiplier, 2);
		} else {
			rightPow = -OscarMath.signumPow(axes.getRotation() * Constants.DrivetrainValues.StickRotateMultiplier, 2);
			leftPow = OscarMath.signumPow(axes.getRotation() * Constants.DrivetrainValues.StickRotateMultiplier, 2);
		}

		return new StoreDriveSpeeds(leftPow, rightPow);
	}

	public StoreDriveSpeeds getTankRotateOnCenterProfile() {
		DriveAxesSupplier axes = oi.driverOI.getGreenbergDriveAxes();
		double rotation;

		if (useFF) {
			rotation = OscarMath.signumPow(getPower(axes.getRotation(), 0.1) * Constants.DrivetrainValues.StickRotateOnCenterMultiplier, 3);
		} else {
			rotation = OscarMath.signumPow(axes.getRotation() * Constants.DrivetrainValues.StickRotateOnCenterMultiplier, 3);
		}

		return new StoreDriveSpeeds(-rotation, rotation);
	}

	private void setSpeeds(double leftPower, double rightPower) {
		this.leftPower = leftPower;
		this.rightPower = rightPower;
	}

	/**
	 * @param acceleration = seconds to max velocity
	 **/
	private double getPower(double stick, double acceleration) {
		double velocity = OscarMath.map(stick, -1, 1, -Motor.kFalcon500.freeSpeed, Motor.kFalcon500.freeSpeed);

		return Constants.DrivetrainValues.kDriveFF.calculate(velocity, velocity/acceleration);
	}

	public class StoreDriveSpeeds {
		public double leftPow, rightPow;

		public StoreDriveSpeeds (double leftPow, double rightPow) {
			this.leftPow = leftPow;
			this.rightPow = rightPow;
		}
	}
}
