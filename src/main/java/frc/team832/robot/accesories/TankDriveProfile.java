package frc.team832.robot.accesories;

import frc.team832.lib.drive.SmartDiffDrive;
import frc.team832.lib.driverinput.oi.DriveAxesSupplier;
import frc.team832.lib.driverinput.oi.SticksDriverOI;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;

import static frc.team832.robot.Robot.oi;

public class TankDriveProfile {
    public double leftPow;
    public double rightPow;
    public SmartDiffDrive.LoopMode loopMode;

    public TankDriveProfile(double leftPow, double rightPow, SmartDiffDrive.LoopMode mode) {
       this.leftPow = leftPow;
       this.rightPow = rightPow;
       this.loopMode = mode;
    }

    public TankDriveProfile() {

    }

    public void calculateTankSpeeds() {
        DriveAxesSupplier axes = oi.driverOI.getTankDriveAxes();
        boolean isRotate = ((SticksDriverOI) oi.driverOI).rightStick.two.get();
        boolean driveStraight = ((SticksDriverOI) oi.driverOI).leftStick.trigger.get() || ((SticksDriverOI) oi.driverOI).rightStick.trigger.get();

        if (driveStraight) {
            TankDriveProfile straight = getTankStraightProfile();
            if (isRotate) {
                TankDriveProfile rotate = getTankRotateProfile();
                this.leftPow = straight.leftPow + rotate.leftPow;
                this.rightPow = straight.rightPow + rotate.rightPow;
                this.loopMode = SmartDiffDrive.LoopMode.PERCENTAGE;
            } else {
                this.leftPow = straight.leftPow;
                this.rightPow = straight.rightPow;
                this.loopMode = SmartDiffDrive.LoopMode.PERCENTAGE;
            }
        } else {
            if (isRotate) {
                TankDriveProfile rotateOnCenter = getTankRotateOnCenterProfile();
                this.leftPow = rotateOnCenter.leftPow;
                this.rightPow = rotateOnCenter.rightPow;
                this.loopMode = SmartDiffDrive.LoopMode.PERCENTAGE;
            } else {
                TankDriveProfile profile = getTankNormalProfile();
                this.leftPow = profile.leftPow;
                this.rightPow = profile.rightPow;
                this.loopMode = SmartDiffDrive.LoopMode.PERCENTAGE;
            }
        }
    }

    public TankDriveProfile getTankStraightProfile() {
        DriveAxesSupplier axes = oi.driverOI.getTankDriveAxes();
        double rightStick = axes.getRight();
        double leftStick = axes.getLeft();
        double power;
        if (((SticksDriverOI) oi.driverOI).leftStick.trigger.get() && ((SticksDriverOI) oi.driverOI).rightStick.trigger.get()) {
            power = (OscarMath.signumPow(rightStick * Constants.DrivetrainValues.stickDriveMultiplier, 2) + OscarMath.signumPow(leftStick * Constants.DrivetrainValues.stickDriveMultiplier, 2)) / 2;
        } else if (((SticksDriverOI) oi.driverOI).rightStick.trigger.get()) {
            power = OscarMath.signumPow(rightStick * Constants.DrivetrainValues.stickDriveMultiplier, 2);
        } else {
            power = (OscarMath.signumPow(leftStick * Constants.DrivetrainValues.stickDriveMultiplier, 2));
        }

        return new TankDriveProfile(power, power, SmartDiffDrive.LoopMode.PERCENTAGE);
    }

    public TankDriveProfile getTankNormalProfile() {
        double rightPower = 0;
        double leftPower = 0;
        DriveAxesSupplier axes = oi.driverOI.getTankDriveAxes();
        double rightStick = axes.getRight();
        double leftStick = axes.getLeft();

        rightPower = OscarMath.signumPow(rightStick * Constants.DrivetrainValues.stickDriveMultiplier, 2);
        leftPower = OscarMath.signumPow(leftStick * Constants.DrivetrainValues.stickDriveMultiplier, 2);

        return new TankDriveProfile(leftPower, rightPower, SmartDiffDrive.LoopMode.PERCENTAGE);
    }

    public TankDriveProfile getTankRotateProfile() {
        double rightPower = 0;
        double leftPower = 0;
        DriveAxesSupplier axes = oi.driverOI.getTankDriveAxes();
        rightPower = -OscarMath.signumPow(axes.getRotation() * Constants.DrivetrainValues.stickRotateMultiplier, 2);
        leftPower = OscarMath.signumPow(axes.getRotation() * Constants.DrivetrainValues.stickRotateMultiplier, 2);

        return new TankDriveProfile(leftPower, rightPower, SmartDiffDrive.LoopMode.PERCENTAGE);
    }

    public TankDriveProfile getTankRotateOnCenterProfile() {
        DriveAxesSupplier axes = oi.driverOI.getTankDriveAxes();
        double rotation = OscarMath.signumPow(axes.getRotation() * Constants.DrivetrainValues.stickRotateOnCenterMultiplier, 3);

        return new TankDriveProfile(-rotation, rotation, SmartDiffDrive.LoopMode.PERCENTAGE);
    }


}
