package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.robot.Constants.SpindexerValues;


public class Spindexer extends SubsystemBase {
	public final boolean initSuccessful;
	private int vibrateCount = 0;

	private final CANSparkMax spinMotor;
	private final ProfiledPIDController spinPID = new ProfiledPIDController(SpindexerValues.SpinkP, 0, 0, SpindexerValues.VelocityConstraints);
	private final ProfiledPIDController positionPID = new ProfiledPIDController(SpindexerValues.PositionkP, 0, 0, SpindexerValues.PositionConstraints);

	private SpinMode spinMode;
	private SpinnerDirection spinDirection = SpinnerDirection.Clockwise;

	public NetworkTableEntry dashboard_RPM, dashboard_targetRPM, dashboard_PIDEffort;

	private double spindexerTargetVelocity = 0, spindexerTargetPosition = 0;

	public Spindexer(GrouchPDP pdp) {
		setName("Spindexer");

		spinMotor = new CANSparkMax(SpindexerValues.SPIN_MOTOR_CAN_ID, Motor.kNEO);
		spinMotor.wipeSettings();
		spinMotor.limitInputCurrent(20);
		spinMotor.setInverted(false); //these might change
		spinMotor.setSensorPhase(true);
		spinMotor.setNeutralMode(NeutralMode.kBrake);

		zeroSpindexer();

		vibrateCount = 0;

		DashboardManager.addTab(this);

		dashboard_RPM = DashboardManager.addTabItem(this, "RPM", 0.0);
		dashboard_targetRPM = DashboardManager.addTabItem(this, "Target RPM", 0.0);
		dashboard_PIDEffort = DashboardManager.addTabItem(this, "PID Effort", 0.0);


		initSuccessful = spinMotor.getCANConnection();// && ballSensor.getDistanceMeters() != 0;
	}

	@Override
	public void periodic() {
	    runSpindexerPID();

	}

	public void setSpinRPM(double rpm, SpinnerDirection spinDirection) {
		this.spinDirection = spinDirection;
		setTargetVelocity(spinDirection == SpinnerDirection.Clockwise ? rpm : -rpm);
	}

	private void setTargetRotation(double rot) {
		spinMode = SpinMode.Position;
		spindexerTargetPosition = rot;
	}

	private void setTargetVelocity(double rpm) {
		spinMode = SpinMode.Velocity;
		spindexerTargetVelocity = rpm;
	}

	public void vibrate() {
		if (vibrateCount > 10) {
			setSpinRPM(30, spinDirection == SpinnerDirection.Clockwise ? SpinnerDirection.CounterClockwise : SpinnerDirection.Clockwise);
			vibrateCount = 0;
			return;
		}
		vibrateCount++;
	}

	public void zeroSpindexer() {
		spinMotor.rezeroSensor();
	}

	public double getRelativeRotations() { return (spinMotor.getSensorPosition() * SpindexerValues.SpinReduction) % 1; }

	public double getVelocity() { return spinMotor.getSensorVelocity() * SpindexerValues.SpinReduction; }


	public void setNeutralMode(NeutralMode mode) {
		spinMotor.setNeutralMode(mode);
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

	public void idle() {
		setTargetVelocity(0);
	}

	public CANSparkMax getSpinMotor() {
		return spinMotor;
	}

	public SpinnerDirection getSpinnerDirection() {
		return spinDirection;
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
