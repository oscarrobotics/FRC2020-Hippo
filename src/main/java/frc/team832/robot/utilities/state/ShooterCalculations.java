package frc.team832.robot.utilities.state;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;

public class ShooterCalculations implements DashboardUpdatable {
    public static double flywheelRPM, exitAngle, visionYaw;
    public static double areaToMeters = 0.0001;

    private static final double CameraAngle = 33.47;
    private static final double PowerPortHeightMeters = 2.5;

    private static NetworkTableEntry dashboard_distance, dashboard_rotation, dashboard_flywheelRPM, dashboard_exitAngle, dashboard_turretYaw;

    static {
        ShooterCalculations me = new ShooterCalculations();
        DashboardManager.addTab(me);
        dashboard_distance = DashboardManager.addTabItem(me, "Distance", 0.0);
        dashboard_rotation = DashboardManager.addTabItem(me, "Rotation", 0.0);
        dashboard_flywheelRPM = DashboardManager.addTabItem(me, "Flywheel RPM", 0.0);
        dashboard_exitAngle = DashboardManager.addTabItem(me, "Hood Exit Angle", 0.0);
        dashboard_turretYaw = DashboardManager.addTabItem(me, "Turret Rotation", 0.0);
    }

    private ShooterCalculations() {

    }

    public static void update(double pitch, Pose2d targetPose, Pose2d robotPose) {
        double distance = targetPose.getTranslation().getDistance(robotPose.getTranslation());
        double rotation = targetPose.getRotation().rotateBy(robotPose.getRotation()).getDegrees() / 360.0; //-targetPose.getRotation().getDegrees() / 360.0;

        flywheelRPM = 5000;//distance < 2 ? 4000 : 8000;
        exitAngle = (pitch - CameraAngle) + 5;//needs testing
        visionYaw = rotation;

        dashboard_distance.setDouble(distance);
        dashboard_rotation.setDouble(rotation);
        dashboard_flywheelRPM.setDouble(flywheelRPM);
        dashboard_turretYaw.setDouble(visionYaw);
        dashboard_exitAngle.setDouble(exitAngle);

    }

    public static void update(double pitch, double yaw, double area) {
        double rotation = yaw / 360.0;
        double distance = PowerPortHeightMeters / Math.tan(pitch - CameraAngle);

        flywheelRPM = 5000;//distance < 2 ? 4000 : 8000;
        exitAngle = pitch + 5;//needs testing
        visionYaw = rotation;//assuming yaw input is -179 to 180

        dashboard_distance.setDouble(distance);
        dashboard_rotation.setDouble(rotation);
        dashboard_flywheelRPM.setDouble(flywheelRPM);
        dashboard_turretYaw.setDouble(visionYaw);
        dashboard_exitAngle.setDouble(exitAngle);
    }

    public static void update(double pitch, double yaw) {
        double distance = PowerPortHeightMeters / Math.tan(pitch - CameraAngle);

        flywheelRPM = 5000;//distance < 2 ? 4000 : 8000;
        exitAngle = pitch + 5;//needs testing
        visionYaw = yaw;

        dashboard_distance.setDouble(distance);
        dashboard_flywheelRPM.setDouble(flywheelRPM);
        dashboard_turretYaw.setDouble(visionYaw);
        dashboard_exitAngle.setDouble(exitAngle);

    }

    public static double getMetersFromArea(double area) {
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
