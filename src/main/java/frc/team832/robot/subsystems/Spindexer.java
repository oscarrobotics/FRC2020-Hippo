package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.*;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.robot.Constants.SpindexerValues;


public class Spindexer extends SubsystemBase {
    public final boolean initSuccessful;
    private int vibrateCount;

    private final CANSparkMax spinMotor;
    private final ProfiledPIDController spinPID = new ProfiledPIDController(SpindexerValues.SpinkP, 0, SpindexerValues.SpinkD, SpindexerValues.VelocityConstraints);
    private final ProfiledPIDController positionPID = new ProfiledPIDController(SpindexerValues.PositionkP, 0, 0, SpindexerValues.PositionConstraints);

    private SpinMode spinMode;
    private SpinnerDirection spinDirection = SpinnerDirection.Clockwise;

    public NetworkTableEntry dashboard_RPM, dashboard_targetRPM, dashboard_PIDEffort, dashboard_ff, dashboard_isStall, dashboard_direction;

    private double spindexerTargetRPM = 0;
    private double spindexerTargetPosition = 0;

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
        dashboard_ff = DashboardManager.addTabItem(this, "FF", 0.0);
        dashboard_isStall = DashboardManager.addTabItem(this, "Stalling", false);
        dashboard_direction = DashboardManager.addTabItem(this, "Direction", SpinnerDirection.Clockwise.toString());

        DashboardManager.getTab(this).add("Spin PID", spinPID);

        initSuccessful = spinMotor.getCANConnection();// && ballSensor.getDistanceMeters() != 0;
    }

    @Override
    public void periodic() {
        runSpindexerPID();
        dashboard_targetRPM.setDouble(spindexerTargetRPM);
        dashboard_RPM.setDouble(getRPM());
        dashboard_isStall.setBoolean(isStalling(10));
        dashboard_direction.setString(spinDirection.toString());
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
        spindexerTargetRPM = rpm;
    }

    public void vibrate(double frequency, double power) {
        if (vibrateCount > frequency * 25) {
            setSpinRPM(power * 100, spinDirection == SpinnerDirection.Clockwise ? SpinnerDirection.CounterClockwise : SpinnerDirection.Clockwise);
            vibrateCount = 0;
            return;
        }
        vibrateCount++;
    }

    public void zeroSpindexer() {
        spinMotor.rezeroSensor();
    }

    public double getRelativeRotations() {
        return (spinMotor.getSensorPosition() * SpindexerValues.SpinReduction) % 1;
    }

    public double getRPM() {
        return spinMotor.getSensorVelocity() * SpindexerValues.SpinReduction;
    }


    public void setNeutralMode(NeutralMode mode) {
        spinMotor.setNeutralMode(mode);
    }

    private void runSpindexerPID() {
        double power;
        if (spinMode == SpinMode.Position) {
            power = positionPID.calculate(getRelativeRotations(), spindexerTargetPosition);
            spinMotor.set(power);
        } else {
            power = spinPID.calculate(getRPM(), spindexerTargetRPM);
            double ff = (0.05 * Math.signum(spindexerTargetRPM)) + (((1 / Motor.kNEO.kv * spindexerTargetRPM) / SpindexerValues.SpinReduction) / spinMotor.getInputVoltage());
            dashboard_ff.setDouble(ff);
            spinMotor.set(spindexerTargetRPM == 0 ? 0 : power + ff);
        }
        dashboard_PIDEffort.setDouble(power);
    }

    public void idle() {
        setTargetVelocity(0);
    }

    public boolean isStalling(double rpmTolerance) {
        return Math.abs(spindexerTargetRPM) > 1 && Math.abs(getRPM()) < rpmTolerance;
    }

    public void switchSpin() {
        setSpinRPM(spindexerTargetRPM, spinDirection == SpinnerDirection.Clockwise ? SpinnerDirection.CounterClockwise : SpinnerDirection.Clockwise);
    }

    public CANSparkMax getSpinMotor() {
        return spinMotor;
    }

    public SpinnerDirection getSpinnerDirection() {
        return spinDirection;
    }

    public Command getAntiJamSpinCommand(double rpm, double resetTime) {
        return new AntiStall(rpm, resetTime);
    }

    public enum SpinnerDirection {
        Clockwise,
        CounterClockwise
    }

    public enum SpinMode {
        Position,
        Velocity
    }

    public class AntiStall extends CommandBase {
        double lastSwitchSec = 0;
        double fpgaSecs;
        double vibrateStartTime;
        final double tolerance;
        final double resetTime;

        public AntiStall(double tolerance, double resetTime) {
            this.tolerance = tolerance;
            this.resetTime = resetTime;
        }

        @Override
        public void execute() {
            fpgaSecs = Timer.getFPGATimestamp();
            if (isStalling(tolerance) && (fpgaSecs - lastSwitchSec >= resetTime)) {
                vibrateStartTime = fpgaSecs;
                if (fpgaSecs - vibrateStartTime < resetTime * 0.8) {
                    vibrate(5, 0.25);
                } else {
                    switchSpin();
                }
                lastSwitchSec = fpgaSecs;
            }
        }
    }
}
