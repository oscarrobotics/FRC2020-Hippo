package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.PDPSlot;
import frc.team832.lib.sensors.REVThroughBorePWM;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;
import frc.team832.robot.Constants.TurretValues;
import frc.team832.robot.utilities.state.ShooterCalculations;

public class Turret extends SubsystemBase {

    public final boolean initSuccessful;
    private double turretTargetDeg;
    private double turretFF = 0;

    private final CANSparkMax motor;
    private final REVThroughBorePWM turretEncoder;
    private final PDPSlot pdpSlot;

    private final KalmanFilter<N2, N1, N1> m_observer = new KalmanFilter<N2, N1, N1>(
            Nat.N2(), Nat.N1(),
            TurretValues.m_turretPlant,
            VecBuilder.fill(2.5, 2.5), // How accurate we think our model is
            VecBuilder.fill(0.0069), // How nice we think our encoder data is
            TurretValues.ControlLoopPeriod);

    private final LinearQuadraticRegulator<N2, N1, N1> m_controller
            = new LinearQuadraticRegulator<N2, N1, N1>(
                    TurretValues.m_turretPlant,
            VecBuilder.fill(120.0, 120.0), // qelms. Velocity error tolerance, in radians per second. Decrease
            // this to more heavily penalize state excursion, or make the controller behave more aggressively.
            VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
            // heavily penalize control effort, or make the controller less aggressive. 12 is a good
            // starting point because that is the (approximate) maximum voltage of a battery.
            TurretValues.ControlLoopPeriod); // Nominal time between loops. 0.020 for TimedRobot, but can be lower if using notifiers.

    private final LinearSystemLoop<N2, N1, N1> turretLoop = new LinearSystemLoop<N2, N1, N1>(
            TurretValues.m_turretPlant,
            m_controller,
            m_observer,
            12.0,
            Constants.TurretValues.ControlLoopPeriod);

    private final Spindexer spindexer;

    private boolean isVision = false;

    private NetworkTableEntry dashboard_turretPos, dashboard_turretPow, dashboard_turretTarget, dashboard_turretError;

    private final PIDController PID = new PIDController(TurretValues.kP, TurretValues.kI, TurretValues.kD);

    public Turret(GrouchPDP pdp, Spindexer spindexer) {
        setName("Turret");
        DashboardManager.addTab(this);
        motor = new CANSparkMax(TurretValues.TURRET_MOTOR_CAN_ID, Motor.kNEO550);
        turretEncoder = new REVThroughBorePWM(TurretValues.TURRET_ENCODER_DIO_CHANNEL);
        pdpSlot = pdp.addDevice(TurretValues.TURRET_PDP_SLOT, motor);

        motor.wipeSettings();
        PID.reset();

        motor.limitInputCurrent(25);
        motor.setNeutralMode(NeutralMode.kBrake);

        PID.setIntegratorRange(-0.05, 0.05);
        PID.setTolerance(2);

        this.spindexer = spindexer;

        // keep turret at init position
        turretTargetDeg = getDegrees();

        dashboard_turretPos = DashboardManager.addTabItem(this, "Position", 0.0);
        dashboard_turretPow = DashboardManager.addTabItem(this, "Power", 0.0);
        dashboard_turretTarget = DashboardManager.addTabItem(this, "Target", 0.0);
        dashboard_turretError = DashboardManager.addTabItem(this, "Error", 0.0);

        initSuccessful = motor.getCANConnection();
    }

    @Override
    public void periodic() {
        runPID();
        dashboard_turretPos.setDouble(getDegrees());
        dashboard_turretPow.setDouble(motor.getOutputVoltage());
        dashboard_turretTarget.setDouble(turretTargetDeg);
        dashboard_turretError.setDouble(PID.getPositionError());
    }

    public void trackTarget(double spindexerRPM) {
        updateFF(spindexerRPM);
        setTurretTargetDegrees(ShooterCalculations.visionYaw + getDegrees(), true);
    }

    protected double calculateSafePosition(boolean isVision, double degrees) {
        double rightBound = isVision ? TurretValues.PracticeTurretRightVisionPosition : TurretValues.PracticeTurretRightPosition;
        double leftBound = isVision ? TurretValues.PracticeTurretLeftVisionPosition : TurretValues.PracticeTurretLeftPosition;

        boolean rightBoundExceeded = degrees > rightBound;
        boolean leftBoundExceeded = degrees < leftBound;

        double safeValue;

        if (rightBoundExceeded) {
            safeValue = rightBound;
        } else if (leftBoundExceeded) {
            safeValue = leftBound;
        } else {
            safeValue = degrees;
        }

        return safeValue;
    }

    private void handleSafety(boolean isVisionMode) {
        double safeTarget = calculateSafePosition(isVisionMode, turretTargetDeg);
        double safeActual = calculateSafePosition(isVisionMode, getDegrees());


        if(safeActual != getDegrees()){
            turretTargetDeg = safeActual;
        } else {
            turretTargetDeg = safeTarget;
        }
    }

    public void setTurretTargetDegrees(double pos, boolean isVision) {
        this.isVision = isVision;
        turretTargetDeg = pos;
    }

    private void updateFF(double spindexerRPM) {
        turretFF =  spindexerRPM * TurretValues.FFMultiplier * (spindexer.getSpinnerDirection() == Spindexer.SpinnerDirection.Clockwise ? -1 : 1);
    }

    private void runPID() {
        handleSafety(isVision);

        double power = PID.calculate(getDegrees(), turretTargetDeg) + turretFF;
        motor.set(power);
        if (PID.getPositionError() <= 1) PID.setI(0);
        else PID.setI(TurretValues.kI);
    }

    public void setForward() { setTurretTargetDegrees(TurretValues.TurretCenterVisionPosition, false); }

    public void setIntake() {
        setTurretTargetDegrees(Constants.TurretValues.IntakeOrientationDegrees, false);
    }

    public void holdTurretPosition() {
        setTurretTargetDegrees(getDegrees(), false);
    }

    double getRotations() {
        return OscarMath.round(turretEncoder.get(), 3);
    }

    double getDegrees() {
        return TurretValues.convertRotationsToDegrees(getRotations());
    }

    public boolean atTargetAngle() {
        return OscarMath.withinEpsilon(5, turretTargetDeg, getDegrees());
    }

    public void stop() {
        motor.set(0);
    }

    public void setNeutralMode(NeutralMode mode) {
        motor.setNeutralMode(mode);
    }

    public void setHeadingSlider(double rightSlider) {
        setTurretTargetDegrees(OscarMath.clipMap(rightSlider,-1, 1, TurretValues.PracticeTurretLeftPosition, TurretValues.PracticeTurretRightPosition), false);
    }

    public void close() {
        turretEncoder.close();
    }
}


