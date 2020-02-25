package frc.team832.robot.utilities.state;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Units;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;

public class ShooterCalculations implements DashboardUpdatable {
    public double flywheelRPM, exitAngle, turretRotation;
    public double areaToMeters = 0.0001;

    private final double CameraAngle = 45;
    private final double PowerPortHeightMeters = 2.5;

    private NetworkTableEntry dashboard_distance, dashboard_rotation, dashboard_flywheelRPM, dashboard_exitAngle, dashboard_turretRotation;

    public ShooterCalculations() {
        dashboard_distance = DashboardManager.addTabItem(this, "Distance", 0.0);
        dashboard_rotation = DashboardManager.addTabItem(this, "Rotation", 0.0);
        dashboard_flywheelRPM = DashboardManager.addTabItem(this, "Flywheel RPM", 0.0);
        dashboard_exitAngle = DashboardManager.addTabItem(this, "Hood Exit Angle", 0.0);
        dashboard_turretRotation = DashboardManager.addTabItem(this, "Turret Rotation", 0.0);
    }

    public void update(double pitch, Pose2d targetPose, Pose2d robotPose) {
        double distance = targetPose.getTranslation().getDistance(robotPose.getTranslation());
        double rotation = targetPose.getRotation().rotateBy(robotPose.getRotation()).getDegrees() / 360.0; //-targetPose.getRotation().getDegrees() / 360.0;

        flywheelRPM = distance < 2 ? 4000 : 8000;
        exitAngle = (pitch - CameraAngle) + 5;//needs testing
        turretRotation = rotation;

        dashboard_distance.setDouble(distance);
        dashboard_rotation.setDouble(rotation);
        dashboard_flywheelRPM.setDouble(flywheelRPM);
        dashboard_turretRotation.setDouble(turretRotation);
        dashboard_exitAngle.setDouble(exitAngle);

    }

    public void update(double pitch, double yaw, double area) {
        double rotation = yaw / 360.0;
        double distance = PowerPortHeightMeters / Math.tan(pitch - CameraAngle);

        flywheelRPM = distance < 2 ? 4000 : 8000;
        exitAngle = pitch + 5;//needs testing
        turretRotation = rotation;//assuming yaw input is -179 to 180

        dashboard_distance.setDouble(distance);
        dashboard_rotation.setDouble(rotation);
        dashboard_flywheelRPM.setDouble(flywheelRPM);
        dashboard_turretRotation.setDouble(turretRotation);
        dashboard_exitAngle.setDouble(exitAngle);
    }

    public double getMetersFromArea(double area) {
        return (1 / area) * areaToMeters; //TODO: change multiplier to make it return distance in meters
    }

    @Override
    public String getDashboardTabName() {
        return "Shooter Calculations";
    }

    @Override
    public void updateDashboardData() {

    }
}
