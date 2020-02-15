package frc.team832.robot.utilities.state;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Units;
import frc.team832.lib.util.OscarMath;

public class ShooterCalculations {
    public double flywheelRPM, exitAngle, turretRotation;
    public double areaToMeters = 0.0001;

    public void update(double pitch, double yaw, double area, Pose2d targetPose, Pose2d robotPose) {
        double distance = targetPose.getTranslation().getDistance(robotPose.getTranslation());
        double rotation = targetPose.getRotation().rotateBy(robotPose.getRotation()).getDegrees() / 360.0;
        //-targetPose.getRotation().getDegrees() / 360.0;
        // (yaw / 360.0);//assuming yaw input is -179 to 180

        flywheelRPM = (distance * 300) + 5000;// (getMetersFromArea(area) * 300) + 5000;//needs testing
        exitAngle = pitch + 5;//needs testing
        turretRotation = rotation;
    }

    public double getMetersFromArea(double area) {
        return (1 / area) * areaToMeters; //TODO: change multiplier to make it return distance in meters
    }

}
