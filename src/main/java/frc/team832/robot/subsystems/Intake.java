package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.motorcontrol2.vendor.CANTalonFX;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.impl.SmartMCAttachedPDPSlot;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;

public class Intake extends SubsystemBase {
	private final CANTalonFX intakeMotor;
	private Solenoid moveIntake;
	SmartMCAttachedPDPSlot intakeSlot;

	NetworkTableEntry dashboard_intakeTargetRPM, dashboard_intakePow, dashboard_intakeRPM;

	public final boolean initSuccessful;

	public Intake(GrouchPDP pdp) {
		setName("Intake");
		//Change Can ID
		intakeMotor = new CANTalonFX(Constants.IntakeValues.INTAKE_MOTOR_CAN_ID);

		intakeSlot = pdp.addDevice(Constants.IntakeValues.INTAKE_MOTOR_PDP_SLOT, intakeMotor);

		intakeMotor.wipeSettings();
		intakeMotor.setInverted(true);

		moveIntake = new Solenoid(Constants.PneumaticsValues.PCM_MODULE_NUM, Constants.PneumaticsValues.INTAKE_SOLENOID_ID);

		DashboardManager.addTab(this);

		dashboard_intakeRPM = DashboardManager.addTabItem(this, "Roller RPM", 0.0);
		dashboard_intakePow = DashboardManager.addTabItem(this, "Power", 0.0);
		dashboard_intakeTargetRPM = DashboardManager.addTabItem(this, "Roller Target RPM", 0.0);


		//Might need to be changed
		intakeMotor.setInverted(false);

		setCurrentLimit(40);

		initSuccessful = intakeMotor.getCANConnection();
	}

	@Override
	public void periodic() {
		dashboard_intakeRPM.setDouble(intakeMotor.getSensorVelocity() * Constants.IntakeValues.IntakeReduction);
	}

	public void intake(double power) {
		intakeMotor.set(power);
	}

	public void outtake(double power) {
		intakeMotor.set(-power);
	}

	public void stop() {
		intakeMotor.set(0);
	}

	public void stopAll() {
		retractIntake();
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
}
