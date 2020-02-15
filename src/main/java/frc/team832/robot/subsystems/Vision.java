package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.vision.ChameleonVisionSubsystem;
import frc.team832.lib.vision.VisionTarget;
import frc.team832.robot.utilities.state.ShooterCalculations;

public class Vision extends ChameleonVisionSubsystem implements DashboardUpdatable {
	private boolean initSuccessful = false;
	private ShooterCalculations calculations = new ShooterCalculations();

	private Drivetrain drivetrain;

	NetworkTableEntry dashboard_pitch, dashboard_yaw, dashboard_area;

	public Vision(Drivetrain drivetrain) {
		super("ACTUALLY DO THIS LATER", 0);//TODO: fix this

		dashboard_area = DashboardManager.addTabItem(this, "Area", 0.0);
		dashboard_pitch = DashboardManager.addTabItem(this, "Pitch", 0.0);
		dashboard_yaw = DashboardManager.addTabItem(this, "Yaw", 0.0);

		this.drivetrain = drivetrain;

		initSuccessful = true;
	}

	public void consumeTarget (VisionTarget target) {
		if (target.isValid)
			calculations.update(target.pitch, target.yaw, target.area, target.poseMeters, drivetrain.getLatestPose());

	}

	public ShooterCalculations getCalculations() {
		return calculations;
	}

	@Override
	public String getDashboardTabName () {
		return "Vision";
	}

	@Override
	public void updateDashboardData () {
		dashboard_area.setDouble(getAreaEntry().getDouble(0.0));
		dashboard_pitch.setDouble(getPitchEntry().getDouble(0.0));
		dashboard_yaw.setDouble(getYawEntry().getDouble(0.0));
	}

    public boolean isInitSuccessful() {
		return initSuccessful;
    }
}

