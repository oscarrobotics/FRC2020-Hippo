package frc.team832.robot.utilities.state;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.util.OscarMath;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class ShooterCalculations {
    public static double flywheelRPM, exitAngle, visionYaw, distance;
    public static double areaToMeters = 0.0001;

    private static final double CameraAngle = 56.53;//-33.47
    private static final double CameraHeight = 0.43;
    private static final double PowerPortHeightMeters = 2.496;

    private PhotonCamera Camera;

    private static NetworkTableEntry dashboard_distance, dashboard_rotation, dashboard_flywheelRPM, dashboard_exitAngle, dashboard_turretYaw, dashboard_spindexerRpm, dashboard_customDistance;

    static {
        ShooterCalculations me = new ShooterCalculations();
        DashboardManager.addTab("ShooterCalc");
        dashboard_distance = DashboardManager.addTabItem("ShooterCalc", "Distance", 0.0);
        dashboard_rotation = DashboardManager.addTabItem("ShooterCalc", "Rotation", 0.0);
        dashboard_flywheelRPM = DashboardManager.addTabItem("ShooterCalc", "Flywheel RPM", 0.0);
        dashboard_exitAngle = DashboardManager.addTabItem("ShooterCalc", "Hood Exit Angle", 0.0);
        dashboard_turretYaw = DashboardManager.addTabItem("ShooterCalc", "Turret Rotation", 0.0);
        dashboard_spindexerRpm = DashboardManager.addTabItem("ShooterCalc", "Spindexer RPM", 0.0);
        dashboard_customDistance = DashboardManager.addTabItem("ShooterCalc", "Our Distance", 0.0);
    }

    public static void update(double pitch, double yaw) {
        double angle = (Math.log(0.4 * distance) * 30) + 65;
        double rpm = (20 * Math.pow(distance - 2, 2)) + (175 * distance) + 5000;

        distance = PhotonUtils.calculateDistanceToTargetMeters(CameraHeight, PowerPortHeightMeters, Math.toRadians(CameraAngle), Math.toRadians(pitch) * 1 / 0.45);
        flywheelRPM = OscarMath.clip(rpm, 0, 7000);
        exitAngle = angle;
        visionYaw = yaw;

        dashboard_distance.setDouble(distance);
        dashboard_flywheelRPM.setDouble(flywheelRPM);
        dashboard_turretYaw.setDouble(visionYaw);
        dashboard_exitAngle.setDouble(exitAngle);
        dashboard_spindexerRpm.setDouble(getSpindexerRpm());

        dashboard_customDistance.setDouble(getDistance(pitch));
    }

    public static double getSpindexerRpm(){
        return (350 * (distance / Math.pow(distance, 2)) + 30);
    }

    public static double getDistance(double pitch) {
        return (PowerPortHeightMeters - CameraHeight) / Math.tan(Math.toRadians(32 + pitch));
    }
}
