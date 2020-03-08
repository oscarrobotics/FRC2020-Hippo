package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.driverstation.dashboard.DashboardWidget;
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


public class Spindexer extends SubsystemBase implements DashboardUpdatable {
	public final boolean initSuccessful;

	private final CANSparkMax spinMotor;
	private final HallEffect hallEffect;
	private final LasersharkDistance ballSensor;
	private final DigitalInput shark;
	private final PIDController spinPID = new PIDController(SpindexerValues.SpinkP, 0, 0);
	private final ProfiledPIDController positionPID = new ProfiledPIDController(SpindexerValues.PositionkP, 0, 0, SpindexerValues.Constraints);

	private final SpindexerStatus spindexerStatus;
	private SpinMode spinMode;

	private final NetworkTableEntry ballSlot0, ballSlot1, ballSlot2, ballSlot3, ballSlot4, dashboard_state, dashboard_hallEffect, dashboard_laserShark;

	private double tempSpindexerRotations = 0;
	private double lastSpinSpeed = 0;
	private double spindexerTargetVelocity = 0, spindexerTargetPosition = 0;

	public Spindexer(GrouchPDP pdp) {
		spinMotor = new CANSparkMax(SpindexerValues.SPIN_MOTOR_CAN_ID, Motor.kNEO);
		spinMotor.wipeSettings();
		setCurrentLimit(10); //this might change
		spinMotor.setInverted(false); //these might change
		spinMotor.setSensorPhase(true);
		spinMotor.setNeutralMode(NeutralMode.kBrake);

		zeroSpindexer();

		spindexerStatus = new SpindexerStatus(pdp, this, spinMotor);

		hallEffect = new HallEffect(SpindexerValues.HALL_EFFECT_DIO_CHANNEL);
		shark = new DigitalInput(2);
		ballSensor = new LasersharkDistance(SpindexerValues.LASERSHARK_DIO_CHANNEL);

//		hallEffect.setupInterrupts(spindexerStatus::onHallEffect);

		hallEffect.setupInterrupts(() -> {
			System.out.println("HALL");
		});

		DashboardManager.addTab(this, this);
		ballSlot0 = DashboardManager.addTabItem(this, "Spindexer/Slot 0", false, DashboardWidget.BooleanBox);
		ballSlot1 = DashboardManager.addTabItem(this, "Spindexer/Slot 1", false, DashboardWidget.BooleanBox);
		ballSlot2 = DashboardManager.addTabItem(this, "Spindexer/Slot 2", false, DashboardWidget.BooleanBox);
		ballSlot3 = DashboardManager.addTabItem(this, "Spindexer/Slot 3", false, DashboardWidget.BooleanBox);
		ballSlot4 = DashboardManager.addTabItem(this, "Spindexer/Slot 4", false, DashboardWidget.BooleanBox);
		dashboard_state = DashboardManager.addTabItem(this, "State", "Default");
		dashboard_hallEffect = DashboardManager.addTabItem(this, "Hall Effect", false, DashboardWidget.BooleanBox);
		dashboard_laserShark = DashboardManager.addTabItem(this, "Laser Shark Meters", 0.0);

		initSuccessful = spinMotor.getCANConnection();// && ballSensor.getDistanceMeters() != 0;
	}

	@Override
	public void periodic() {
	    runSpindexerPID();
	    spindexerStatus.update();
	}

	@Override
	public void updateDashboardData() {
		ballSlot0.setBoolean(spindexerStatus.getSlot(0));
		ballSlot1.setBoolean(spindexerStatus.getSlot(1));
		ballSlot2.setBoolean(spindexerStatus.getSlot(2));
		ballSlot3.setBoolean(spindexerStatus.getSlot(3));
		ballSlot4.setBoolean(spindexerStatus.getSlot(4));
		dashboard_state.setString(spindexerStatus.state.toString());
		dashboard_hallEffect.setBoolean(getHallEffect());
		dashboard_laserShark.setDouble(getBallSensorMeters());
	}

	public void setDumbPosition(double rot) {
		setTargetRotation(rot);
	}

	public void setDumbRPM(double rpm) {
		setTargetVelocity(rpm);
	}

	public void setSpinRPM(double rpm, SpinnerDirection spinDirection) {
		setSpinRPM(rpm, spinDirection, false, 0, 0);
	}

	public void setSpinRPM(double rpm, SpinnerDirection spinDirection, boolean waitForShooter, double shooterCurrentRPM, double shooterTargetRPM) {
		if (!waitForShooter || Math.abs(shooterCurrentRPM - shooterTargetRPM) < 100) {
			lastSpinSpeed = rpm;
			if (spinDirection == SpinnerDirection.Clockwise) {
				setTargetVelocity(rpm);
			}
			else {
				setTargetVelocity(-rpm);
			}
		}
	}

	public void switchSpin() {
		setSpinRPM(lastSpinSpeed, spindexerStatus.getSpinDirection() == SpinnerDirection.Clockwise ? SpinnerDirection.CounterClockwise : SpinnerDirection.Clockwise);
	}

	public void setToPocket(BallPosition position) { setTargetRotation(position.rotations); }

	public void setToEmpty() {
		int pos;
		if(getState() != SpindexerStatus.SpindexerState.FULL){
			pos = spindexerStatus.getFirstEmpty();
			var empty = intToPosition(pos);
			setTargetRotation(empty.rotations);
		}
	}

	public void setTargetRotation(double rot) {
		spinMode = SpinMode.Position;
		spindexerTargetPosition = rot;
	}

	public void setTargetVelocity(double rpm) {
		spinMode = SpinMode.Velocity;
		spindexerTargetVelocity = rpm;
	}

	public boolean getHallEffect() {
		return !hallEffect.get();
	}

	public void zeroSpindexer() {
		spinMotor.rezeroSensor();
	}

	public SpindexerStatus.SpindexerState getState() { return spindexerStatus.getState(); }

	public SpinnerDirection getSpinnerDirection() {
		return spindexerStatus.getSpinDirection();
	}

	public boolean isStalled() {
		return spindexerStatus.isStalling();
	}

	public double getRelativeRotations() { return (spinMotor.getSensorPosition() * SpindexerValues.SpinReduction) % 1; }

	public double getAbsoluteRotations() { return spindexerStatus.getAbsoluteRotations(); }

	public double getVelocity() { return spinMotor.getSensorVelocity() * SpindexerValues.SpinReduction; }

	public void stopSpin() { setTargetVelocity(0); }

	public void spinCounterclockwise(double pow) {
		spinMotor.set(-OscarMath.clip(pow, 0, 1));
	}

	public void spinClockwise(double pow) {
		spinMotor.set(OscarMath.clip(pow, 0, 1));
	}

	public boolean isSensorOverSlot() {
		double sensorOffset = -0.02;
		return Math.abs(getRelativeRotations() - BallPosition.Position0.rotations + sensorOffset) < 0.05
				|| Math.abs(getRelativeRotations() - BallPosition.Position1.rotations + sensorOffset) < 0.05
				|| Math.abs(getRelativeRotations() - BallPosition.Position2.rotations + sensorOffset) < 0.05
				|| Math.abs(getRelativeRotations() - BallPosition.Position3.rotations + sensorOffset) < 0.05
				|| Math.abs(getRelativeRotations() - BallPosition.Position4.rotations + sensorOffset) < 0.05;
	}

	public double getBallSensorRaw() {
		return ballSensor.getPercentageDistance();
	}

	public boolean isBall() {
		return getBallSensorMeters() <= 0.15 && getBallSensorMeters() >= 0.02;
	}

	public double getBallSensorMeters() {
		return ballSensor.getDistanceMeters();
	}

	public boolean isFull() {
		return spindexerStatus.isFull();
	}

	public void setCurrentLimit(int currentLimit) {
		spinMotor.limitInputCurrent(currentLimit);
	}

	public void setNeutralMode(NeutralMode mode) {
		spinMotor.setNeutralMode(mode);
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

	private void runSpindexerPID() {
		double power;
		if (spinMode == SpinMode.Position) {
			power = positionPID.calculate(getRelativeRotations(), spindexerTargetPosition);
		} else {
			power = spinPID.calculate(getVelocity(), spindexerTargetVelocity);
		}
		spinMotor.set(power);
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

	public void idle() {
		setTargetVelocity(0);
	}

	@Override
	public String getDashboardTabName() {
		return "Spindexer";
	}

	public CANSparkMax getSpinMotor() {
		return spinMotor;
	}

	public enum SpinnerDirection {
		Clockwise,
		CounterClockwise
	}

	public enum SpinMode {
		Position,
		Velocity
	}
}
