package frc.team832.robot;

import frc.team832.lib.control.PDP;
import frc.team832.robot.subsystems.Drivetrain;
import frc.team832.robot.subsystems.Intake;

public class RobotContainer {

	public static final Drivetrain drivetrain = new Drivetrain();
	public static final Intake intake = new Intake();
//	public static final Vision vision;
	public static final PDP pdp = new PDP(0);

	public boolean init() {
		if(drivetrain.isInitSuccessful() && intake.isInitSuccessful()) {
			return true;
		} else {
			return false;
		}
	}

}