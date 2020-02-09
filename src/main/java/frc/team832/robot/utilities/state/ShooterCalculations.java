package frc.team832.robot.utilities.state;


import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;

public class ShooterCalculations {
    public double flywheelRPM, hoodPosition, turretPosition;
    public double HoodMinAngle = 10;
    public double HoodMaxAngle = 80;
    public double HoodMinTicks = 0;
    public double HoodMaxTicks = 1000;

    public void update() {
        double distanceMeters = 0, pitch = 0, yaw = 0;

        hoodPosition = getHoodPositionFromAngle(pitch + 5);//needs testing
        flywheelRPM = (distanceMeters * 300) + 5000;//needs testing
        turretPosition = Constants.ShooterValues.TurretPowerTrain.calculateTicksFromPosition(yaw / 180);//assuming yaw input is -179 to 180
    }

    private double getHoodPositionFromAngle(double angle) {
        return OscarMath.map(angle, HoodMinAngle, HoodMaxAngle, HoodMinTicks, HoodMaxTicks);
    }

}
