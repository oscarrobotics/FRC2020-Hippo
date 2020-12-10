package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    private boolean isVision = false;

    private NetworkTableEntry dashboard_turretPos, dashboard_turretPow, dashboard_turretTarget, dashboard_turretError;

    private final PIDController PID = new PIDController(TurretValues.kP, TurretValues.kI, TurretValues.kD);

    public Turret(GrouchPDP pdp) {
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
        setTurretTargetDegrees(ShooterCalculations.visionYaw + ((spindexerRPM / 30.0) * Math.signum(spindexerRPM)) + getDegrees(), true);
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
        setVisionMode(isVision);
        turretTargetDeg = pos;
    }

    private void updateFF(double spindexerRPM) {
        turretFF =  spindexerRPM * TurretValues.FFMultiplier;
    }

    private void runPID() {
        handleSafety(isVision);
        double power = PID.calculate(getDegrees(), turretTargetDeg);
        motor.set(power); //+ (Math.signum(power) * turretFF)
        if (PID.getPositionError() <= 1) PID.setI(0);
        else PID.setI(TurretValues.kI);
    }

    public void setForward(boolean isVision) { setTurretTargetDegrees(TurretValues.TurretCenterVisionPosition, isVision); }

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

    private boolean atTarget() {
        return Math.abs(getDegrees() - ShooterCalculations.visionYaw) < 1;
    }

    private void setVisionMode(boolean visionMode){
        isVision = visionMode;
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
}


