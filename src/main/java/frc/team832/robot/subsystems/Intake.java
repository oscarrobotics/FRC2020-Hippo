package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.impl.SmartMCAttachedPDPSlot;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;

public class Intake extends SubsystemBase implements DashboardUpdatable {
	private final CANSparkMax intakeMotor;
	private Solenoid moveIntake;
	SmartMCAttachedPDPSlot intakeSlot;

	NetworkTableEntry dashboard_intakeTargetRPM, dashboard_intakePow, dashboard_intakeRPM;

	public final boolean initSuccessful;

	public Intake(GrouchPDP pdp) {
		//Change Can ID
		intakeMotor = new CANSparkMax(Constants.IntakeValues.INTAKE_MOTOR_CAN_ID, Motor.kNEO550);

		intakeSlot = pdp.addDevice(Constants.IntakeValues.INTAKE_MOTOR_PDP_SLOT, intakeMotor);

		intakeMotor.wipeSettings();

		moveIntake = new Solenoid(Constants.PneumaticsValues.PCM_MODULE_NUM, Constants.PneumaticsValues.INTAKE_SOLENOID_ID);

		DashboardManager.addTab(this, this);

		dashboard_intakeRPM = DashboardManager.addTabItem(this, "Roller RPM", 0.0);
		dashboard_intakePow = DashboardManager.addTabItem(this, "Power", 0.0);
		dashboard_intakeTargetRPM = DashboardManager.addTabItem(this, "Roller Target RPM", 0.0);


		//Might need to be changed
		intakeMotor.setInverted(false);

		setCurrentLimit(40);

		initSuccessful = intakeMotor.getCANConnection();
	}

	@Override
	public void updateDashboardData() {
		dashboard_intakeRPM.setDouble(intakeMotor.getSensorVelocity() * Constants.IntakeValues.IntakeReduction);
	}

	public void intake(double power) {
		OscarMath.clip(power, 0, 1);
		intakeMotor.set(power);
	}

	public void outtake(double power) {
		OscarMath.clip(power, 0, 1);
		intakeMotor.set(-power);
	}

	public void setIntakeRPM(double rpm) {
		double motorRPM = rpm * Constants.IntakeValues.IntakeReduction;
		double pow = Constants.IntakeValues.FF.calculate(OscarMath.clip(motorRPM, 0, Motor.kNEO550.freeSpeed));
		intakeMotor.set(pow);
		dashboard_intakePow.setDouble(pow);
		dashboard_intakeTargetRPM.setDouble(rpm);
	}

	public void setOuttakeRPM(double rpm) {
		double motorRPM = -rpm * Constants.IntakeValues.IntakeReduction;
		double pow = Constants.IntakeValues.FF.calculate(OscarMath.clip(motorRPM, -Motor.kNEO550.freeSpeed, 0));
		intakeMotor.set(pow);
		dashboard_intakePow.setDouble(pow);
		dashboard_intakeTargetRPM.setDouble(motorRPM);
	}

	public void stop() {
		intakeMotor.set(0);
	}

	public void extendIntake() {
		moveIntake.set(true);
	}

	public void retractIntake() {
		moveIntake.set(false);
	}

	public void setCurrentLimit(int amps) {
		intakeMotor.limitInputCurrent(amps);
	}

	@Override
	public String getDashboardTabName() {
		return "Intake";
	}

	public void setDumbPower(double leftSlider) {
		extendIntake();
		intake(OscarMath.map(leftSlider,-1.0, 1.0, 0.0, 1.0));
	}
}
