package frc.team832.robot.accesories;

import frc.team832.lib.drive.SmartDiffDrive;

public class ArcadeDriveProfile {
    public double xPow;
    public double rotPow;
    public SmartDiffDrive.LoopMode loopMode;

    public ArcadeDriveProfile(double xPow, double rotPow, SmartDiffDrive.LoopMode mode) {
        this.xPow = xPow;
        this.rotPow = rotPow;
        this.loopMode = mode;
    }
}
