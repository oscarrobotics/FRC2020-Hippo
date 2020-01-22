package frc.team832.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;
import frc.team832.robot.accesories.SpindexerStatus;
import frc.team832.robot.commands.teleop.TemplateCommand;

import java.util.ArrayList;
import java.util.List;

public class Spindexer extends SubsystemBase {
	private boolean initSuccessful = false;

	private final CANSparkMax spinMotor, feedMotor;
	private final DigitalInput hallEffect;
	private SpindexerStatus spindexerStatus = new SpindexerStatus();
	private final List<Boolean> ballStatus = new ArrayList<>();

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
		spindexerStatus.update(ballStatus);
	}

	public void setCurrentLimit(int currentLimit) {
		spinMotor.limitInputCurrent(currentLimit);
		feedMotor.limitInputCurrent(currentLimit);
	}

	public void spinClockwise(double pow) {
		spinMotor.set(OscarMath.clip(pow, 0, 1));
	}

	public void stopSpin() {
		spinMotor.set(0);
	}

	public void stopFeed() {
		feedMotor.set(0);
	}

	public void spinCounterclockwise(double pow) {
		spinMotor.set(-OscarMath.clip(pow, 0, 1));
	}

	public void feed(double pow) {
		feedMotor.set(pow);
	}

	public void setPosition(int pos) {
		spinMotor.setPosition(pos);
	}
	
	public double getPosition() {
		return spinMotor.getSensorPosition();
	}

	public void setFeederProfileVelocity(double velocity) {
		feedMotor.setVelocity(velocity);
	}

	public void setSpinProfileVelocity(double velocity) {
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

	public List<Boolean> getBallPositions() { return spindexerStatus.getStateArray(); }

	public SpindexerStatus.SpindexerState getState() { return spindexerStatus.getState(); }
}
