package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;
import frc.team832.robot.utilities.positions.BallPosition;
import frc.team832.robot.utilities.positions.SafePosition;
import frc.team832.robot.utilities.state.SpindexerStatus;

import java.util.ArrayList;
import java.util.List;

import static frc.team832.robot.Constants.SpindexerValues.SpinPowertrain;
import static frc.team832.robot.Constants.SpindexerValues.SpinReduction;
import static frc.team832.robot.Robot.*;

public class Spindexer extends SubsystemBase {
	private boolean initSuccessful = false;
	private double tempSpindexerRotations = 0;
	private double lastSpinSpeed = 0;

	private final CANSparkMax spinMotor;
	private final DigitalInput hallEffect;
	private SpindexerStatus spindexerStatus;
	private final List<Boolean> ballStatus = new ArrayList<>();
	private PIDController spinPID = new PIDController(Constants.SpindexerValues.SPIN_kP, 0, Constants.SpindexerValues.SPIN_kD);
	private ProfiledPIDController positionPID = new ProfiledPIDController(Constants.SpindexerValues.POSITION_kP, 0, Constants.SpindexerValues.POSITION_kD, Constants.SpindexerValues.Constraints);

	public Spindexer(GrouchPDP pdp) {
		spinMotor = new CANSparkMax(Constants.SpindexerValues.SPIN_MOTOR_CAN_ID, Motor.kNEO);

		spinMotor.wipeSettings();

		setCurrentLimit(30); //this might change

		spinMotor.setInverted(false); //these might change

		spinMotor.setSensorPhase(true);

		spinMotor.setNeutralMode(NeutralMode.kBrake);

		hallEffect = new DigitalInput(Constants.SpindexerValues.HALL_EFFECT_CHANNEL);

		spindexerStatus = new SpindexerStatus(pdp, this, spinMotor);

		initSuccessful = true;
	}

	@Override
	public void periodic() {
		spindexerStatus.update(ballStatus, getHallEffect(), spinMotor.getSensorVelocity());
	}

	public void setCurrentLimit(int currentLimit) {
		spinMotor.limitInputCurrent(currentLimit);
	}

	public boolean isSafe() {
		return Math.abs(shooter.getTurretRotations() - spindexer.getRelativeRotations()) <= .05;
	}

	public void stopSpin() {
		spinMotor.set(0);
	}

	public void spinCounterclockwise(double pow) {
		spinMotor.set(-OscarMath.clip(pow, 0, 1));
	}

	public void spinClockwise(double pow) {
		spinMotor.set(OscarMath.clip(pow, 0, 1));
	}

	public boolean isFull() {
//		return spindexerStatus.isFull();
		return false;
	}

	public enum SpinnerDirection {
		Clockwise,
		CounterClockwise;
	}

	public SpinnerDirection getSpinnerDirection() {
		return spindexerStatus.getSpinDirection();
	}

	public void setSpinRPM(double rpm, SpinnerDirection spinDirection) {
		lastSpinSpeed = rpm;
		if (spinDirection == SpinnerDirection.Clockwise) {
			spinMotor.set(spinPID.calculate(spinMotor.getSensorVelocity(), rpm));
		}
		else {
			spinMotor.set(spinPID.calculate(spinMotor.getSensorVelocity(), -rpm));
		}
	}

	public boolean isStalled() {
		return spindexerStatus.isStalling();
	}

	public void setTargetPosition(double pos) {
		spinMotor.set(positionPID.calculate(spinMotor.getSensorPosition(), SpinPowertrain.calculateTicksFromPosition(pos)));
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

	public SpindexerStatus.SpindexerState getState() { return spindexerStatus.getState(); }

	public double getRelativeRotations() {
		return spinMotor.getSensorPosition() * SpinReduction;
	}

	public double getAbsoluteRotations() {
		return spindexerStatus.getAbsoluteRotations() + getRelativeRotations(); }

	public void switchSpin() {
		setSpinRPM(lastSpinSpeed, spindexerStatus.getSpinDirection() == SpinnerDirection.Clockwise ? SpinnerDirection.CounterClockwise : SpinnerDirection.Clockwise);
	}

	public boolean isUnloaded() {
		if (tempSpindexerRotations == 0) {
			tempSpindexerRotations = getAbsoluteRotations();
		}
		if (Math.abs(tempSpindexerRotations - getAbsoluteRotations()) > 2) {
			tempSpindexerRotations = 0;
			return true;
		}
		return false;
	}

	public void setToPocket(BallPosition position) {
		double target = SpinPowertrain.calculateTicksFromPosition(position.value);
		spinMotor.set(positionPID.calculate(spinMotor.getSensorPosition(), target));
	}

	public void setToSafeSpot() {
		double target = SpinPowertrain.calculateTicksFromPosition(getNearestSafeSpotRelativeToFeeder());
		spinMotor.set(positionPID.calculate(spinMotor.getSensorPosition(), target));
	}

	public void setToEmpty() {
		int pos;
		if(getState() != SpindexerStatus.SpindexerState.FULL){
			pos = spindexerStatus.getFirstEmpty();
			var empty = intToPosition(pos);
			setTargetPosition(empty.value);
		}
	}

	public double getNearestSafeSpotRelativeToFeeder() {
		return superStructure.calculateSpindexerPosRelativeToFeeder(getNearestSafePosition().value);
	}

	private SafePosition getNearestSafePosition() {
		double pos = spinMotor.getSensorPosition();
		double nearestDistance = SpinPowertrain.calculateTicksFromPosition(0.1);
		SafePosition safePosition = SafePosition.Position1;

		if (Math.abs(pos - SpinPowertrain.calculateTicksFromPosition(SafePosition.Position1.value)) < nearestDistance) {
			safePosition = SafePosition.Position1;
		} else if (Math.abs(pos - SpinPowertrain.calculateTicksFromPosition(SafePosition.Position2.value)) < nearestDistance) {
			safePosition = SafePosition.Position2;
		} else if (Math.abs(pos - SpinPowertrain.calculateTicksFromPosition(SafePosition.Position3.value)) < nearestDistance) {
			safePosition = SafePosition.Position3;
		} else if (Math.abs(pos - SpinPowertrain.calculateTicksFromPosition(SafePosition.Position4.value)) < nearestDistance) {
			safePosition = SafePosition.Position4;
		} else if (Math.abs(pos - SpinPowertrain.calculateTicksFromPosition(SafePosition.Position5.value)) < nearestDistance) {
			safePosition = SafePosition.Position5;
		}

		return safePosition;
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
}