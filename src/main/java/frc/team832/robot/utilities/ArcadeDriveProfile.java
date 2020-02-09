package frc.team832.robot.utilities;

import frc.team832.lib.drive.SmartDiffDrive;
import frc.team832.lib.driverinput.oi.DriveAxesSupplier;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;

import static frc.team832.robot.Robot.oi;

public class ArcadeDriveProfile {
    public double xPow;
    public double rotPow;
    public SmartDiffDrive.LoopMode loopMode;

    public void calculateArcadeSpeeds() {
        double xPower = 0;
        double rotPower = 0;
        DriveAxesSupplier axes = oi.driverOI.getArcadeDriveAxes();
        xPower = OscarMath.signumPow(-axes.getRightY() * Constants.DrivetrainValues.StickDriveMultiplier, 3);
        rotPower = OscarMath.signumPow(axes.getLeftY() * Constants.DrivetrainValues.StickDriveMultiplier, 3);

        this.xPow = xPower;
        this.rotPow = rotPower;
        this.loopMode = SmartDiffDrive.LoopMode.PERCENTAGE;
    }
}
