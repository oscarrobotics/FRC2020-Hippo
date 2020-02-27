package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.driverstation.dashboard.DashboardWidget;
import frc.team832.lib.vision.ChameleonVisionSubsystem;
import frc.team832.lib.vision.VisionTarget;
import frc.team832.robot.utilities.state.ShooterCalculations;

public class Vision extends ChameleonVisionSubsystem implements DashboardUpdatable {
	public final boolean initSuccessful;

	private final Drivetrain drivetrain;
	private final NetworkTableEntry dashboard_pitch, dashboard_yaw, dashboard_area, dashboard_isValid;
	private VisionTarget target = new VisionTarget();
//	private ShooterCalculations calculations = new ShooterCalculations();

	public Vision(Drivetrain drivetrain) {
		super("USB Camera-B4.09.24.1", 0.05);//TODO: fix this

		DashboardManager.addTab(this, this);

		dashboard_area = DashboardManager.addTabItem(this, "Area", 0.0);
		dashboard_pitch = DashboardManager.addTabItem(this, "Pitch", 0.0);
		dashboard_yaw = DashboardManager.addTabItem(this, "Yaw", 0.0);
		dashboard_isValid = DashboardManager.addTabItem(this, "Is Target Valid", false, DashboardWidget.BooleanBox);

		this.drivetrain = drivetrain;

		initSuccessful = true;
	}

	@Override
	public void periodic(){
		updateDashboardData();
		updateVision();
		consumeTarget(target);
	}

	private void updateVision() {
		target.isValid = getIsValidEntry().getBoolean(false);
		target.pitch = getPitchEntry().getDouble(0);
		target.yaw = getYawEntry().getDouble(0);
	}

	@Override
	public void consumeTarget (VisionTarget target) {
		if (target.isValid) {
			ShooterCalculations.update(target.pitch, target.yaw);
		}
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
		dashboard_isValid.setBoolean(getIsValidEntry().getBoolean(false));
	}
}

