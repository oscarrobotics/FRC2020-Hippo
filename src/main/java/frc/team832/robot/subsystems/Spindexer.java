package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;
import frc.team832.robot.accesories.SpindexerStatus;

import java.util.ArrayList;
import java.util.List;

public class Spindexer extends SubsystemBase {
	private boolean initSuccessful = false;
	private SpinnerDirection currentSpinDirection;
	private int spindexerRotations = 0;
	private int tempSpindexerRotations = 0;
	private double lastSpinSpeed = 0;

	private final CANSparkMax spinMotor, feedMotor;
	private final DigitalInput hallEffect;
	private SpindexerStatus spindexerStatus = new SpindexerStatus();
	private final List<Boolean> ballStatus = new ArrayList<>();
	public PIDController feedPID = new PIDController(Constants.SpindexerValues.FEED_kP, 0, Constants.SpindexerValues.FEED_kD);
	public PIDController spinPID = new PIDController(Constants.SpindexerValues.SPIN_kP, 0, Constants.SpindexerValues.SPIN_kD);



	public Spindexer() {
		spinMotor = new CANSparkMax(Constants.SpindexerValues.SPIN_MOTOR_CAN_ID, Motor.kNEO);
		feedMotor = new CANSparkMax(Constants.SpindexerValues.FEED_MOTOR_CAN_ID, Motor.kNEO);

		spinMotor.wipeSettings();
		feedMotor.wipeSettings();

		setCurrentLimit(30); //this might change

		spinMotor.setInverted(false); //these might change
		feedMotor.setInverted(false);

		spinMotor.setSensorPhase(true);
		feedMotor.setSensorPhase(true);

		feedMotor.setNeutralMode(NeutralMode.kCoast);
		spinMotor.setNeutralMode(NeutralMode.kBrake);

		hallEffect = new DigitalInput(Constants.SpindexerValues.HALL_EFFECT_CHANNEL);

		initSuccessful = true;
	}

	@Override
	public void periodic() {
		if(getHallEffect()) {
			zeroSpindexer();
			if (currentSpinDirection == SpinnerDirection.Clockwise) spindexerRotations++;
			else spindexerRotations--;
		}
		spindexerStatus.update(ballStatus);
	}

	public void setCurrentLimit(int currentLimit) {
		spinMotor.limitInputCurrent(currentLimit);
		feedMotor.limitInputCurrent(currentLimit);
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
		currentSpinDirection = SpinnerDirection.CounterClockwise;
	}

	public void spinClockwise(double pow) {
		spinMotor.set(OscarMath.clip(pow, 0, 1));
		currentSpinDirection = SpinnerDirection.Clockwise;
	}

	public void feed(double pow) {
		feedMotor.set(pow);
	}

	public void setFeedRPM(double rpm) {
		feedPID.calculate(feedMotor.getSensorVelocity(), rpm);
	}

	public enum SpinnerDirection {
		Clockwise,
		CounterClockwise;
	}

	public SpinnerDirection getSpinnerDirection() {
		return currentSpinDirection;
	}

	public void setSpinRPM(double rpm, SpinnerDirection spinDirection) {
		lastSpinSpeed = rpm;
		if (spinDirection == SpinnerDirection.Clockwise) {
			spinMotor.set(spinPID.calculate(spinMotor.getSensorVelocity(), rpm));
			currentSpinDirection = SpinnerDirection.Clockwise;
		}
		else {
			spinMotor.set(spinPID.calculate(spinMotor.getSensorVelocity(), -rpm));
			currentSpinDirection = SpinnerDirection.CounterClockwise;
		}
	}

	public boolean isStalled() {
		// TODO: fix stall detection
		return false;
	}

	public void setTargetPosition(int pos) {
		spinMotor.set(spinPID.calculate(spinMotor.getSensorPosition(), Constants.SpindexerValues.SpinPowertrain.calculateTicksFromPosition(pos)));
	}
	
	public double getPosition() {
		return spinMotor.getSensorPosition();
	}

	public void zeroSpindexer() {
		spinMotor.rezeroSensor();
	}

	public boolean getHallEffect() {
		return hallEffect.get();
	}

	public double getRelativeFeederPosition() {
		return feedMotor.getSensorPosition();
	}

	public double getAbsoluteFeederPosition() { return getRelativeFeederPosition() + spindexerRotations; }

	public boolean isInitSuccessful() {
		return initSuccessful;
	}

	public List<Boolean> getBallPositions() { return spindexerStatus.getBooleanList(); }

	public SpindexerStatus.SpindexerState getState() { return spindexerStatus.getState(); }

	public boolean atFeedRpm() {
		return Math.abs(feedMotor.getSensorVelocity() - Constants.SpindexerValues.FEED_RPM) < 100;
	}

	public void switchSpin() {
		setSpinRPM(lastSpinSpeed, currentSpinDirection == SpinnerDirection.Clockwise ? SpinnerDirection.CounterClockwise : SpinnerDirection.Clockwise);
	}

	public boolean isUnloaded() {
		if (tempSpindexerRotations == 0) {
			tempSpindexerRotations = spindexerRotations;
		}
		if (Math.abs(tempSpindexerRotations - spindexerRotations) > 2) {
			tempSpindexerRotations = 0;
			return true;
		}
		return false;
	}

	public void setToEmpty() {
		int pos;
		if(getState() != SpindexerStatus.SpindexerState.FULL){
			pos = spindexerStatus.getFirstEmpty();
			setTargetPosition((int)intToPosition(pos).getValue());
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
