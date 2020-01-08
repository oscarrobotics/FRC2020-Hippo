package frc.team832.robot.accesories;

import frc.team832.lib.drive.SmartDiffDrive;

public class TankDriveProfile {
    public double leftPow;
    public double rightPow;
    public SmartDiffDrive.LoopMode loopMode;

    public TankDriveProfile(double leftPow, double rightPow, SmartDiffDrive.LoopMode mode) {
       this.leftPow = leftPow;
       this.rightPow = rightPow;
       this.loopMode = mode;
    }
}
