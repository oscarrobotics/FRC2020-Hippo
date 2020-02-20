package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.sensors.digital.HallEffect;
import frc.team832.lib.sensors.digital.LasersharkDistance;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants.SpindexerValues;
import frc.team832.robot.utilities.positions.BallPosition;
import frc.team832.robot.utilities.state.SpindexerStatus;

public class Spindexer extends SubsystemBase {
	public final boolean initSuccessful;

	private final CANSparkMax spinMotor;
	private final HallEffect hallEffect;
	private final LasersharkDistance ballSensor;
	private final SpindexerStatus spindexerStatus;
	private final PIDController spinPID = new PIDController(SpindexerValues.SpinkP, 0, 0);
	private final ProfiledPIDController positionPID = new ProfiledPIDController(SpindexerValues.PositionkP, 0, 0, SpindexerValues.Constraints);

	private double tempSpindexerRotations = 0;
	private double lastSpinSpeed = 0;

	public Spindexer(GrouchPDP pdp) {
		spinMotor = new CANSparkMax(SpindexerValues.SPIN_MOTOR_CAN_ID, Motor.kNEO);
		spinMotor.wipeSettings();
		setCurrentLimit(30); //this might change
		spinMotor.setInverted(false); //these might change
		spinMotor.setSensorPhase(true);
		spinMotor.setNeutralMode(NeutralMode.kBrake);

		hallEffect = new HallEffect(SpindexerValues.HALL_EFFECT_DIO_CHANNEL);
		ballSensor = new LasersharkDistance(SpindexerValues.LASERSHARK_DIO_CHANNEL);

		spindexerStatus = new SpindexerStatus(pdp, this, spinMotor);

		hallEffect.setupInterrupts(spindexerStatus::onHallEffect);

		initSuccessful = spinMotor.getCANConnection() && ballSensor.getDistanceMeters() != 0;
	}

	@Override
	public void periodic() {
		spindexerStatus.update();
	}

	public double getBallSensorRaw() {
		return ballSensor.getPercentageDistance();
	}

	public boolean getBallSensor() {
		return getBallSensorRaw() >= Units.inchesToMeters(3);
	}

	public void setCurrentLimit(int currentLimit) {
		spinMotor.limitInputCurrent(currentLimit);
	}

	public boolean isSafe() {
		return Math.abs(getNearestSafeRotationRelativeToFeeder() - getRelativeRotations()) < .05;
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
		return spindexerStatus.isFull();
	}

    public void setNeutralMode(NeutralMode mode) {
		spinMotor.setNeutralMode(mode);
    }

    public enum SpinnerDirection {
		Clockwise,
		CounterClockwise;
	}

	public SpinnerDirection getSpinnerDirection() {
		return spindexerStatus.getSpinDirection();
	}

	public void setSpinRPM(double rpm, SpinnerDirection spinDirection) {
		setSpinRPM(rpm, spinDirection, false, 0, 0);
	}

	public void setSpinRPM(double rpm, SpinnerDirection spinDirection, boolean waitForShooter, double shooterCurrentRPM, double shooterTargetRPM) {
		if (!waitForShooter || Math.abs(shooterCurrentRPM - shooterTargetRPM) < 100) {
			lastSpinSpeed = rpm;
			if (spinDirection == SpinnerDirection.Clockwise) {
				spinMotor.set(spinPID.calculate(spinMotor.getSensorVelocity(), rpm));
			}
			else {
				spinMotor.set(spinPID.calculate(spinMotor.getSensorVelocity(), -rpm));
			}
		}
	}

	public boolean isStalled() {
		return spindexerStatus.isStalling();
	}

	public void setTargetPosition(double pos) {
		spinMotor.set(positionPID.calculate(spinMotor.getSensorPosition(), SpindexerValues.SpinPowertrain.calculateTicksFromPosition(pos)));
	}
	
	public double getPosition() {
		return spinMotor.getSensorPosition();
	}

	public void zeroSpindexer() {
		spinMotor.rezeroSensor();
	}

	public SpindexerStatus.SpindexerState getState() { return spindexerStatus.getState(); }

	public double getRelativeRotations() {
		return spinMotor.getSensorPosition() * SpindexerValues.SpinReduction;
	}

	public double getAbsoluteRotations() {
		return spindexerStatus.getAbsoluteRotations(); }

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

	public boolean isOverBallSlot() {
		return Math.abs(getRelativeRotations() - getNearestBallRotationRelativeToFeeder()) < 0.05;
	}

	public boolean isOverBallPosition(BallPosition position) {
		return Math.abs(getRelativeRotations() - position.rotations) < 0.05;
	}

	public void setToPocket(BallPosition position) {
		double target = SpindexerValues.SpinPowertrain.calculateTicksFromPosition(position.rotations);
		spinMotor.set(positionPID.calculate(spinMotor.getSensorPosition(), target));
	}

	public void setToSafeSpot() {
		double target = SpindexerValues.SpinPowertrain.calculateTicksFromPosition(getNearestSafeRotationRelativeToFeeder());
		spinMotor.set(positionPID.calculate(spinMotor.getSensorPosition(), target));
	}

	public void setToEmpty() {
		int pos;
		if(getState() != SpindexerStatus.SpindexerState.FULL){
			pos = spindexerStatus.getFirstEmpty();
			var empty = intToPosition(pos);
			setTargetPosition(empty.rotations);
		}
	}

	// YOU CAN'T DO THIS! No cross-subsystem references except *inside* the SuperStructure!!!
	public double getNearestSafeRotationRelativeToFeeder() {
		return 0;
//		return getNearestBallRotationRelativeToFeeder() + superStructure.calculateSpindexerRotRelativeToFeeder(0.1);
	}

	// YOU CAN'T DO THIS! No cross-subsystem references except *inside* the SuperStructure!!!
	public double getNearestBallRotationRelativeToFeeder() {
		return 0;
//		return superStructure.calculateSpindexerRotRelativeToFeeder(getNearestBallPosition().rotations);
	}

	public BallPosition getNearestBallPosition() {
		double pos = spinMotor.getSensorPosition();
		double nearestDistance = SpindexerValues.SpinPowertrain.calculateTicksFromPosition(0.1);
		BallPosition ballPosition = BallPosition.Position0;

		if (Math.abs(pos - SpindexerValues.SpinPowertrain.calculateTicksFromPosition(BallPosition.Position0.rotations)) < nearestDistance) {
			ballPosition = BallPosition.Position0;
		} else if (Math.abs(pos - SpindexerValues.SpinPowertrain.calculateTicksFromPosition(BallPosition.Position1.rotations)) < nearestDistance) {
			ballPosition = BallPosition.Position1;
		} else if (Math.abs(pos - SpindexerValues.SpinPowertrain.calculateTicksFromPosition(BallPosition.Position2.rotations)) < nearestDistance) {
			ballPosition = BallPosition.Position2;
		} else if (Math.abs(pos - SpindexerValues.SpinPowertrain.calculateTicksFromPosition(BallPosition.Position3.rotations)) < nearestDistance) {
			ballPosition = BallPosition.Position3;
		} else if (Math.abs(pos - SpindexerValues.SpinPowertrain.calculateTicksFromPosition(BallPosition.Position4.rotations)) < nearestDistance) {
			ballPosition = BallPosition.Position4;
		}

		return ballPosition;
	}

	private BallPosition intToPosition(int i) {
		if(i == 0) {
			return BallPosition.Position0;
		} else if(i == 1) {
			return BallPosition.Position1;
		} else if (i == 2) {
			return BallPosition.Position2;
		} else if (i == 3) {
			return BallPosition.Position3;
		} else {
			return BallPosition.Position4;
		}
	}
}
