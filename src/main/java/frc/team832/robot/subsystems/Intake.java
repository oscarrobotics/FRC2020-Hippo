package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.impl.SmartMCAttachedPDPSlot;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;

public class Intake extends SubsystemBase {
	private final CANSparkMax intakeMotor;
	private Solenoid moveIntake;
	SmartMCAttachedPDPSlot intakeSlot;

	public final boolean initSuccessful;

	public Intake(GrouchPDP pdp) {
		//Change Can ID
		intakeMotor = new CANSparkMax(Constants.IntakeValues.INTAKE_MOTOR_CAN_ID, Motor.kNEO550);

		intakeSlot = pdp.addDevice(Constants.IntakeValues.INTAKE_MOTOR_PDP_SLOT, intakeMotor);

		intakeMotor.wipeSettings();

		moveIntake = new Solenoid(Constants.PneumaticsValues.PCM_MODULE_NUM, Constants.PneumaticsValues.INTAKE_SOLENOID_ID);


		//Might need to be changed
		intakeMotor.setInverted(false);

		setCurrentLimit(40);

		initSuccessful = intakeMotor.getCANConnection();
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
		OscarMath.clip(rpm, 0, Motor.kNEO550.freeSpeed);
		intakeMotor.set(Constants.IntakeValues.FF.calculate(rpm));
	}

	public void setOuttakeRPM(double rpm) {
		OscarMath.clip(rpm, -Motor.kNEO550.freeSpeed, 0);
		intakeMotor.set(Constants.IntakeValues.FF.calculate(-rpm));
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

}
