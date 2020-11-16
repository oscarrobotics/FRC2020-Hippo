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
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;

public class Vision extends SubsystemBase implements DashboardUpdatable {
	public final boolean initSuccessful;

	private final Drivetrain drivetrain;

	private final PhotonCamera gloworm = new PhotonCamera("gloworm");

	private PhotonTrackedTarget target;
	private boolean hasTargets;

	public Vision(Drivetrain drivetrain) {
		DashboardManager.addTab(this, this);
		this.drivetrain = drivetrain;
		initSuccessful = true;
	}

	@Override
	public void periodic(){
		updateDashboardData();
		updateVision();
	}

	private void updateVision() {
		PhotonPipelineResult latestResult = gloworm.getLatestResult();
		hasTargets = latestResult.hasTargets();
		if (hasTargets) {
			target = latestResult.getBestTarget();
			ShooterCalculations.update(target.getPitch(), target.getYaw());
		}
	}

	public PhotonTrackedTarget getTarget() {
		return target;
	}

	public void driverMode(boolean enable) {
		gloworm.setDriverMode(enable);
	}

	@Override
	public String getDashboardTabName () {
		return "Vision";
	}

	@Override
	public void updateDashboardData () {}

	public boolean hasTarget() {
		return hasTargets;
	}
}


