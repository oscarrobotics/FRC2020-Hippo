package frc.team832.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.robot.Constants;
import frc.team832.robot.commands.TemplateCommand;

public class Spindexer extends SubsystemBase {
	private boolean initSuccessful = false;

	private final CANSparkMax spinMotor, feedMotor;
	private final DigitalInput hallEffect;
	private boolean[] ballPositions = {false, false, false, false, false};

	public Spindexer() {
		spinMotor = new CANSparkMax(Constants.SpindexerValues.SPIN_MOTOR_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
		feedMotor = new CANSparkMax(Constants.SpindexerValues.FEED_MOTOR_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

		spinMotor.wipeSettings();
		feedMotor.wipeSettings();

		setCurrentLimit(30); //this might change

		spinMotor.setInverted(false); //these might change
		feedMotor.setInverted(false);

		hallEffect = new DigitalInput(Constants.SpindexerValues.HALL_EFFECT_CHANNEL);

		setDefaultCommand(new TemplateCommand(this));

		initSuccessful = true;
	}

	@Override
	public void periodic() {
		if(getHallEffect()) {
			zeroSpindexer();
		}
	}

	public void setCurrentLimit(int currentLimit) {
		spinMotor.limitInputCurrent(currentLimit);
		feedMotor.limitInputCurrent(currentLimit);
	}

	public void spin(double pow) {
		spinMotor.set(pow);
	}

	public void feed(double pow) {
		feedMotor.set(pow);
	}

	public void setSpindexerPosition(int pos) {
		spinMotor.setPosition(pos);
	}
	
	public double getSpindexerPosition() {
		return spinMotor.getSensorPosition();
	}

	public void setFeederVelocity(double velocity) {
		feedMotor.setVelocity(velocity);
	}

	public void setSpinVelocity(double velocity) {
		spinMotor.setVelocity(velocity);
	}

	public void zeroSpindexer() {
		spinMotor.rezeroSensor();
	}

	private boolean getHallEffect() {
		return hallEffect.get();
	}

	public double getFeederPosition() {
		return feedMotor.getSensorPosition();
	}

	public void holdFeederPosition() {
		feedMotor.setPosition(getFeederPosition());
	}
	public boolean isInitSuccessful() {
		return initSuccessful;
	}

	public boolean[] getBallPositions() {
		return ballPositions;
	}
}
