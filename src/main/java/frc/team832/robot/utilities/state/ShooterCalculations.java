package frc.team832.robot.utilities.state;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class ShooterCalculations {
    public static double flywheelRPM, exitAngle, visionYaw;
    public static double areaToMeters = 0.0001;

    private static final double CameraAngle = 56.53;
    private static final double CameraHeight = 0.43;
    private static final double PowerPortHeightMeters = 2.44;

    private PhotonCamera Camera;

    private static NetworkTableEntry dashboard_distance, dashboard_rotation, dashboard_flywheelRPM, dashboard_exitAngle, dashboard_turretYaw;

    static {
        ShooterCalculations me = new ShooterCalculations();
        DashboardManager.addTab("ShooterCalc");
        dashboard_distance = DashboardManager.addTabItem("ShooterCalc", "Distance", 0.0);
        dashboard_rotation = DashboardManager.addTabItem("ShooterCalc", "Rotation", 0.0);
        dashboard_flywheelRPM = DashboardManager.addTabItem("ShooterCalc", "Flywheel RPM", 0.0);
        dashboard_exitAngle = DashboardManager.addTabItem("ShooterCalc", "Hood Exit Angle", 0.0);
        dashboard_turretYaw = DashboardManager.addTabItem("ShooterCalc", "Turret Rotation", 0.0);
    }

    public static void update(double pitch, double yaw) {
        double distance = PhotonUtils.calculateDistanceToTargetMeters(CameraHeight, PowerPortHeightMeters, Math.toRadians(CameraAngle), Math.toRadians(pitch));
        double angle = Math.log((0.5 * distance) + 1) * 40;

        flywheelRPM = 7000;
        exitAngle = angle + 52;
        visionYaw = yaw;

        dashboard_distance.setDouble(distance);
        dashboard_flywheelRPM.setDouble(flywheelRPM);
        dashboard_turretYaw.setDouble(visionYaw);
        dashboard_exitAngle.setDouble(exitAngle);
    }
}
