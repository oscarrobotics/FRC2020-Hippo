package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.robot.Constants;

import java.security.cert.CertificateParsingException;

public class Pneumatics extends SubsystemBase implements DashboardUpdatable {
	boolean initSuccessful;
	private Solenoid intakeSolenoid;
	private Solenoid propUpSolenoid;
	private Solenoid wheelOfFortuneSolenoid;


	public Pneumatics() {
		DashboardManager.addTab(this);
		DashboardManager.getTab(this).add(this);

		this.intakeSolenoid = new Solenoid(Constants.PneumaticsValues.PCM_MODULE_NUM, Constants.PneumaticsValues.INTAKE_SOLENOID_ID);
		this.propUpSolenoid = new Solenoid(Constants.PneumaticsValues.PCM_MODULE_NUM, Constants.PneumaticsValues.PROP_UP_SOLENOID_ID);
		this.wheelOfFortuneSolenoid = new Solenoid(Constants.PneumaticsValues.PCM_MODULE_NUM, Constants.PneumaticsValues.WHEEL_O_FORTUNE_SOLENOID_ID);

		this.initSuccessful = true;
	}

	public void extendIntake() {
		intakeSolenoid.set(true);
	}

	public void retractIntake() {
		intakeSolenoid.set(false);
	}

	public void propUp() {
		propUpSolenoid.set(true);
	}

	public void retractProp() {
		propUpSolenoid.set(false);
	}

	@Override
	public String getDashboardTabName () {
		return null;
	}

	@Override
	public void updateDashboardData () {

	}
}
