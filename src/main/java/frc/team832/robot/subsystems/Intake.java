package frc.team832.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;
import frc.team832.robot.commands.teleop.TemplateCommand;

public class Intake extends SubsystemBase {
	private boolean initSuccessful = false;

	private final CANSparkMax intakeMotor;

	public PIDController PID = new PIDController(Constants.IntakeValues.kP,0, Constants.IntakeValues.kD);

	public Intake() {
		//Change Can ID
		intakeMotor = new CANSparkMax(Constants.IntakeValues.INTAKE_MOTOR_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

		intakeMotor.wipeSettings();

		//Might will needed to be changed
		intakeMotor.setInverted(false);

		setCurrentLimit(40);

		setDefaultCommand(new TemplateCommand(this));

		initSuccessful = true;
	}

	public boolean isInitSuccessful() {
		return initSuccessful;
	}

	public void setCurrentLimit(int amps) {
		intakeMotor.limitInputCurrent(amps);
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
		OscarMath.clip(rpm, 0, 15000);
		intakeMotor.set(PID.calculate(intakeMotor.getSensorVelocity(), rpm));
	}

	public void setOuttakeRPM(double rpm) {
		OscarMath.clip(rpm, 0, 15000);
		intakeMotor.set(PID.calculate(intakeMotor.getSensorVelocity(), -rpm));
	}

	public void stop() {
		intakeMotor.set(0);
	}
}
