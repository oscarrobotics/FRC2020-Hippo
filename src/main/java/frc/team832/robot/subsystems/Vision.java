package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.robot.utilities.state.ShooterCalculations;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
	public final boolean initSuccessful;

	private final PhotonCamera gloworm = new PhotonCamera("gloworm");

	private PhotonTrackedTarget target;
	private boolean hasTargets;

	public Vision() {
		setName("Vision");
		DashboardManager.addTab(this);
		initSuccessful = true;
	}

	@Override
	public void periodic(){
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

	public boolean hasTarget() {
		return hasTargets;
	}
}


