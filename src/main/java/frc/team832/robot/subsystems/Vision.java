package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.vision.ChameleonVisionSubsystem;
import frc.team832.lib.vision.VisionTarget;
import frc.team832.robot.utilities.state.ShooterCalculations;

public class Vision extends ChameleonVisionSubsystem implements DashboardUpdatable {
	private boolean initSuccessful = false;
	private ShooterCalculations calculations = new ShooterCalculations();

	public Vision() {
		super("ACTUALLY DO THIS LATER", 0);//TODO: fix this
		initSuccessful = true;
	}

	public void consumeTarget (VisionTarget target) {
		if (target.isValid) {
			calculations.update(target.pitch, target.yaw, target.area);
		}
	}

	@Override
	public String getDashboardTabName () {
		return "Vision";
	}

	@Override
	public void updateDashboardData () {

	}

    public boolean isInitSuccessful() {
		return initSuccessful;
    }
}

