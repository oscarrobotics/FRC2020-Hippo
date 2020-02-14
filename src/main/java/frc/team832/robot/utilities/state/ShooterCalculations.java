package frc.team832.robot.utilities.state;

import frc.team832.lib.util.OscarMath;

public class ShooterCalculations {
    public double flywheelRPM, exitAngle, turretRotation;

    public void update(double pitch, double yaw, double area) {

        exitAngle = pitch + 5;//needs testing
        flywheelRPM = (getMetersFromArea(area) * 300) + 5000;//needs testing
        turretRotation = (yaw / 360.0);//assuming yaw input is -179 to 180
    }

    public double getMetersFromArea(double area) {
        return (1 / area) * 0.001; //TODO: change multiplier to make it return distance in meters
    }

}
