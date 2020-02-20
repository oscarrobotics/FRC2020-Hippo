package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.robot.Constants;

public class Pneumatics extends SubsystemBase implements DashboardUpdatable {
	public final boolean initSuccessful;
	private final Solenoid moveIntake;
	private final Solenoid propUpRobot;
	private final Solenoid wheelOfFortunePneumatics;
	private final Solenoid climbLock;

	public Pneumatics() {
		DashboardManager.addTab(this, this);

		moveIntake = new Solenoid(Constants.PneumaticsValues.PCM_MODULE_NUM, Constants.PneumaticsValues.INTAKE_SOLENOID_ID);
		propUpRobot = new Solenoid(Constants.PneumaticsValues.PCM_MODULE_NUM, Constants.PneumaticsValues.PROP_UP_SOLENOID_ID);
		wheelOfFortunePneumatics = new Solenoid(Constants.PneumaticsValues.PCM_MODULE_NUM, Constants.PneumaticsValues.WHEEL_O_FORTUNE_SOLENOID_ID);
		climbLock = new Solenoid(Constants.PneumaticsValues.PCM_MODULE_NUM, Constants.PneumaticsValues.CLIMB_LOCK_SOLENOID_ID);

		initSuccessful = true;
	}

	public void extendIntake() {
		moveIntake.set(true);
	}

	public void retractIntake() {
		moveIntake.set(false);
	}

	public void propUp() {
		propUpRobot.set(true);
	}

	public void retractProp() {
		propUpRobot.set(false);
	}

	public void extendWOFManipulator() {
		wheelOfFortunePneumatics.set(true);
	}

	public void retractWOFManipulator() {
		wheelOfFortunePneumatics.set(true);
	}

	public void lockClimb() {
		climbLock.set(true);
	}

	public void unlockClimb() {
		climbLock.set(false);
	}

	@Override
	public String getDashboardTabName () {
		return "Pneumatics";
	}

	@Override
	public void updateDashboardData () {}
}
