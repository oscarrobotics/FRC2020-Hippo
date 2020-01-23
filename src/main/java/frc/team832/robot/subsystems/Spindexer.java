package frc.team832.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.PIDController;
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
	public PIDController feedPID = new PIDController(Constants.SpindexerValues.FEED_kP, 0, Constants.SpindexerValues.FEED_kD);
	public PIDController spinPID = new PIDController(Constants.SpindexerValues.SPIN_kP, 0, Constants.SpindexerValues.SPIN_kD);


	public Spindexer() {
		spinMotor = new CANSparkMax(Constants.SpindexerValues.SPIN_MOTOR_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
		feedMotor = new CANSparkMax(Constants.SpindexerValues.FEED_MOTOR_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

		spinMotor.wipeSettings();
		feedMotor.wipeSettings();

		setCurrentLimit(30); //this might change

		spinMotor.setInverted(false); //these might change
		feedMotor.setInverted(false);

		spinMotor.setSensorPhase(true);
		feedMotor.setSensorPhase(true);

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

	public void stopAll() {
		stopSpin();
		stopFeed();
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

	public void setFeedRPM(double rpm) {
		feedPID.calculate(feedMotor.getSensorVelocity(), rpm);
	}

	public void setClockwiseRPM(double rpm) {
		OscarMath.clip(rpm, 0, 6000);
		spinMotor.set(spinPID.calculate(spinMotor.getSensorVelocity(), rpm));
	}

	public void setCounterclockwiseRPM(double rpm) {
		OscarMath.clip(rpm, 0, 6000);
		spinMotor.set(spinPID.calculate(spinMotor.getSensorVelocity(), -rpm));
	}

	public void setPosition(double pos) {
		spinMotor.set(spinPID.calculate(spinMotor.getSensorPosition(), pos));
	}
	
	public double getPosition() {
		return spinMotor.getSensorPosition();
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

	public boolean isInitSuccessful() {
		return initSuccessful;
	}

	public List<Boolean> getBallPositions() { return spindexerStatus.getBooleanList(); }

	public SpindexerStatus.SpindexerState getState() { return spindexerStatus.getState(); }

	public boolean atFeedRpm() {
		return Math.abs(spinMotor.getSensorVelocity() - Constants.SpindexerValues.FEED_RPM) < 100;
	}

	public void setToEmpty() {
		int pos;
		if(getState() != SpindexerStatus.SpindexerState.FULL){
			pos = spindexerStatus.getFirstEmpty();
			setPosition(intToPosition(pos).getValue());
		}

	}

	private BallPosition intToPosition(int i) {
		if(i == 0) {
			return BallPosition.Position1;
		} else if(i == 1) {
			return BallPosition.Position2;
		} else if (i == 2) {
			return BallPosition.Position3;
		} else if (i == 3) {
			return BallPosition.Position4;
		} else {
			return BallPosition.Position5;
		}
	}

	public enum BallPosition {
		Position1(Constants.SpindexerValues.SpinPowertrain.calculateTicksFromPosition(.2*0)),
		Position2(Constants.SpindexerValues.SpinPowertrain.calculateTicksFromPosition(.2*1)),
		Position3(Constants.SpindexerValues.SpinPowertrain.calculateTicksFromPosition(.2*2)),
		Position4(Constants.SpindexerValues.SpinPowertrain.calculateTicksFromPosition(.2*3)),
		Position5(Constants.SpindexerValues.SpinPowertrain.calculateTicksFromPosition(.2*4));

		double value;
		private BallPosition(double value) {
			this.value = value;
		}

		public double getValue() {
			return value;
		}
	}
}
