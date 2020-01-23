package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.vision.VisionTarget;
import frc.team832.robot.accesories.VisionProfile;

public class Vision extends SubsystemBase implements DashboardUpdatable {
	public Vision() { super(); }

	public void consumeTarget (VisionTarget target) {}

	public VisionProfile getProfile () {
		return new VisionProfile(0, 0, 0);
	}

	@Override
	public String getDashboardTabName () {
		return null;
	}

	@Override
	public void updateDashboardData () {

	}
}

