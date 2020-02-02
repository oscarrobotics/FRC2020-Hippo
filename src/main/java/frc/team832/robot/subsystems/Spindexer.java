package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.PDPBreaker;
import frc.team832.lib.power.PDPSlot;
import frc.team832.lib.power.monitoring.StallDetector;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;
import frc.team832.robot.Robot;
import frc.team832.robot.accesories.SpindexerStatus;

import java.util.ArrayList;
import java.util.List;

public class Spindexer extends SubsystemBase {
	private boolean initSuccessful = false;
	private SpinnerDirection currentSpinDirection;
	private int spindexerRotations = 0;
	private int tempSpindexerRotations = 0;
	private double lastSpinSpeed = 0;

	private final CANSparkMax spinMotor;
	private final DigitalInput hallEffect;
	private SpindexerStatus spindexerStatus = new SpindexerStatus();
	private final List<Boolean> ballStatus = new ArrayList<>();
	private PIDController spinPID = new PIDController(Constants.SpindexerValues.SPIN_kP, 0, Constants.SpindexerValues.SPIN_kD);
	private ProfiledPIDController positionPID = new ProfiledPIDController(Constants.SpindexerValues.POSITION_kP, 0, Constants.SpindexerValues.POSITION_kD, Constants.SpindexerValues.Constraints);
	private StallDetector spinStall = new StallDetector(new PDPSlot(Robot.pdp.getBasePDP(), Constants.SpindexerValues.SPIN_MOTOR_PDP_SLOT, PDPBreaker.ThirtyAmp));



	public Spindexer() {
		spinMotor = new CANSparkMax(Constants.SpindexerValues.SPIN_MOTOR_CAN_ID, Motor.kNEO);

		spinMotor.wipeSettings();

		setCurrentLimit(30); //this might change

		spinMotor.setInverted(false); //these might change

		spinMotor.setSensorPhase(true);

		spinMotor.setNeutralMode(NeutralMode.kBrake);

		hallEffect = new DigitalInput(Constants.SpindexerValues.HALL_EFFECT_CHANNEL);

		spinStall.setMinStallMillis(500);
		spinStall.setStallCurrent(30);

		initSuccessful = true;
	}

	@Override
	public void periodic() {
		if(getHallEffect()) {
			zeroSpindexer();
			if (currentSpinDirection == SpinnerDirection.Clockwise) spindexerRotations++;
			else spindexerRotations--;
		}
		spinStall.updateStallStatus();
		spindexerStatus.update(ballStatus);
	}

	public void setCurrentLimit(int currentLimit) {
		spinMotor.limitInputCurrent(currentLimit);
	}

	public void stopAll() {
		stopSpin();
	}

	public void stopSpin() {
		spinMotor.set(0);
	}

	public void spinCounterclockwise(double pow) {
		spinMotor.set(-OscarMath.clip(pow, 0, 1));
		currentSpinDirection = SpinnerDirection.CounterClockwise;
	}

	public void spinClockwise(double pow) {
		spinMotor.set(OscarMath.clip(pow, 0, 1));
		currentSpinDirection = SpinnerDirection.Clockwise;
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
		return spinStall.getStallStatus().isStalled;
	}

	public void setTargetPosition(double pos) {
		spinMotor.set(positionPID.calculate(spinMotor.getSensorPosition(), Constants.SpindexerValues.SpinPowertrain.calculateTicksFromPosition(pos)));
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

	public boolean isInitSuccessful() {
		return initSuccessful;
	}

	public List<Boolean> getBallPositions() { return spindexerStatus.getBooleanList(); }

	public SpindexerStatus.SpindexerState getState() { return spindexerStatus.getState(); }

	public double getRelativeSpinPosition () {
		return spinMotor.getSensorPosition();
	}

	public double getAbsoluteSpinPosition () {
		return getRelativeSpinPosition() + spindexerRotations; }

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
			setTargetPosition(intToPosition(pos).value);
		}
	}

	public SafePosition getNearestSafePosition() {
		if (Math.abs(spinMotor.getSensorPosition() - Constants.SpindexerValues.SpinPowertrain.calculateTicksFromPosition(SafePosition.Position1.value)) < Constants.SpindexerValues.SpinPowertrain.calculateTicksFromPosition(0.1)) {
			return SafePosition.Position1;
		} else if (Math.abs(spinMotor.getSensorPosition() - Constants.SpindexerValues.SpinPowertrain.calculateTicksFromPosition(SafePosition.Position2.value)) < Constants.SpindexerValues.SpinPowertrain.calculateTicksFromPosition(0.1)) {
			return SafePosition.Position2;
		} else if (Math.abs(spinMotor.getSensorPosition() - Constants.SpindexerValues.SpinPowertrain.calculateTicksFromPosition(SafePosition.Position3.value)) < Constants.SpindexerValues.SpinPowertrain.calculateTicksFromPosition(0.1)) {
			return SafePosition.Position3;
		} else if (Math.abs(spinMotor.getSensorPosition() - Constants.SpindexerValues.SpinPowertrain.calculateTicksFromPosition(SafePosition.Position4.value)) < Constants.SpindexerValues.SpinPowertrain.calculateTicksFromPosition(0.1)) {
			return SafePosition.Position4;
		} else (Math.abs(spinMotor.getSensorPosition() - Constants.SpindexerValues.SpinPowertrain.calculateTicksFromPosition(SafePosition.Position5.value)) < Constants.SpindexerValues.SpinPowertrain.calculateTicksFromPosition(0.1)) {
			return SafePosition.Position5;
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
		Position1(0.0),
		Position2(0.2),
		Position3(0.4),
		Position4(0.6),
		Position5(0.8);

		double value;
		BallPosition(double value) {
			this.value = value;
		}
	}

	public enum SafePosition {
		Position1(0.1),
		Position2(0.3),
		Position3(0.5),
		Position4(0.7),
		Position5(0.9);

		double value;
		SafePosition(double value) {
			this.value = value;
		}
	}
}
