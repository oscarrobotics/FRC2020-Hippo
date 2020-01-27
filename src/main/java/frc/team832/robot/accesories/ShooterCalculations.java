package frc.team832.robot.accesories;


import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;

public class ShooterCalculations {
    public double flywheelRPM, hoodPosition, turretPosition;

    public void calculate(double distanceMeters, double pitch, double yaw) {
        hoodPosition = getHoodPositionFromAngle(pitch + 5);
        flywheelRPM = (distanceMeters * 300) + Constants.ShooterValues.BASE_SHOOTING_RPM;
        turretPosition = Constants.ShooterValues.TurretPowerTrain.calculateTicksFromPosition(yaw / 180);//assuming yaw input is -179 to 180
    }

    private double getHoodPositionFromAngle(double angle) {
        return OscarMath.map(angle, Constants.ShooterValues.HOOD_MIN_ANGLE, Constants.ShooterValues.HOOD_MAX_ANGLE, 0, 1);
    }
}
