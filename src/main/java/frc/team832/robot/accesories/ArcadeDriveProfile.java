package frc.team832.robot.accesories;

import frc.team832.lib.drive.SmartDiffDrive;
import frc.team832.lib.driverinput.oi.DriveAxesSupplier;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;

import static frc.team832.robot.Robot.oi;

public class ArcadeDriveProfile {
    public double xPow;
    public double rotPow;
    public SmartDiffDrive.LoopMode loopMode;

    public ArcadeDriveProfile(double xPow, double rotPow, SmartDiffDrive.LoopMode mode) {
        this.xPow = xPow;
        this.rotPow = rotPow;
        this.loopMode = mode;
    }

    public ArcadeDriveProfile() {

    }

    public void calculateArcadeSpeeds() {
        double xPower = 0;
        double rotPower = 0;
        DriveAxesSupplier axes = oi.driverOI.getArcadeDriveAxes();
        xPower = OscarMath.signumPow(-axes.getRight() * Constants.DrivetrainValues.StickDriveMultiplier, 3);
        rotPower = OscarMath.signumPow(axes.getLeft() * Constants.DrivetrainValues.StickDriveMultiplier, 3);

        this.xPow = xPower;
        this.rotPow = rotPower;
        this.loopMode = SmartDiffDrive.LoopMode.PERCENTAGE;
    }
}
