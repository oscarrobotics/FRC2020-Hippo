package frc.team832.robot.utilities.state;

import frc.team832.lib.util.OscarMath;

public class ShooterCalculations {
    public double flywheelRPM, exitAngle, turretRotation;

    public void update() {
        double distance = 0, pitch = 0, yaw = 0;//get values from vision

        exitAngle = pitch + 5;//needs testing
        flywheelRPM = (distance * 300) + 5000;//needs testing
        turretRotation = yaw / 180.0;//assuming yaw input is -179 to 180
    }


}
