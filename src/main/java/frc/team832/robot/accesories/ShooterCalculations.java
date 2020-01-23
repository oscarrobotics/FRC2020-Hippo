package frc.team832.robot.accesories;


import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;

public class ShooterCalculations {
    public double flywheelRPM, hoodAngle, turretHeading;

    public void calculate(double distanceMeters, double pitch, double yaw) {
        hoodAngle = OscarMath.map(pitch, 0, 90, Constants.ShooterValues.HOOD_MIN_ANGLE, Constants.ShooterValues.HOOD_MAX_ANGLE);
        flywheelRPM = (distanceMeters * 300) + 5000;
        turretHeading = yaw;
    }
}
