package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.vision.VisionTarget;

public class Vision extends SubsystemBase implements DashboardUpdatable {
	private boolean initSuccessful = false;
	public Vision() {
		super();
		initSuccessful = true;
	}

	public void consumeTarget (VisionTarget target) {}

	@Override
	public String getDashboardTabName () {
		return null;
	}

	@Override
	public void updateDashboardData () {

	}

    public boolean isInitSuccessful() {
		return initSuccessful;
    }
}

