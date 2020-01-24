package frc.team832.robot.accesories;


import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;

public class ShooterCalculations {
    public double flywheelRPM, hoodPosition, turretPosition;

    public void calculate(double distanceMeters, double pitch, double yaw) {
        hoodPosition = OscarMath.map(pitch, Constants.ShooterValues.HOOD_MIN_ANGLE, Constants.ShooterValues.HOOD_MAX_ANGLE, Constants.ShooterValues.HOOD_MIN_TICKS, Constants.ShooterValues.HOOD_MAX_TICKS);
        flywheelRPM = (distanceMeters * 300) + 5000;
        turretPosition = Constants.ShooterValues.TurretPowerTrain.calculateTicksFromPosition(yaw / 180);
    }
}
