package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.PDPSlot;
import frc.team832.lib.sensors.REVThroughBorePWM;
import frc.team832.robot.Constants;
import frc.team832.robot.Constants.TurretValues;
import frc.team832.robot.utilities.state.ShooterCalculations;

public class Turret extends SubsystemBase implements DashboardUpdatable {

    public final boolean initSuccessful;
    private double turretTargetDeg;

    private final CANSparkMax motor;
    private final REVThroughBorePWM encoder;
    private final PDPSlot pdpSlot;

    private boolean isVision = false;

    private NetworkTableEntry dashboard_turretPos, dashboard_turretPow, dashboard_turretTarget;

    private final PIDController PID = new PIDController(TurretValues.TurretkP, 0, 0);

    public Turret(GrouchPDP pdp) {
        DashboardManager.addTab(this, this);
        motor = new CANSparkMax(TurretValues.TURRET_MOTOR_CAN_ID, Motor.kNEO550);
        encoder = new REVThroughBorePWM(TurretValues.TURRET_ENCODER_DIO_CHANNEL);
        pdpSlot = pdp.addDevice(TurretValues.TURRET_PDP_SLOT, motor);

        motor.wipeSettings();
        PID.reset();

        motor.limitInputCurrent(25);
        motor.setNeutralMode(NeutralMode.kBrake);


        // keep turret at init position
        turretTargetDeg = getRotations();

        dashboard_turretPos = DashboardManager.addTabItem(this, "Position", 0.0);
        dashboard_turretPow = DashboardManager.addTabItem(this, "Power", 0.0);
        dashboard_turretTarget = DashboardManager.addTabItem(this, "Target", 0.0);

        initSuccessful = motor.getCANConnection();
    }

    @Override
    public void periodic() {
        runPID();
    }

    @Override
    public void updateDashboardData() {
        dashboard_turretPos.setDouble(getDegrees());
        dashboard_turretPow.setDouble(motor.getOutputVoltage());
        dashboard_turretTarget.setDouble(turretTargetDeg);
    }

    public void setTurretTargetDegrees(double pos, boolean isVision) {
        setVisionMode(isVision);
        turretTargetDeg = pos;
    }

    public void holdTurretPosition() {
        setTurretTargetDegrees(getDegrees(), false);
    }

    double getRotations() {
        return encoder.get();
    }

    double getDegrees() {
        return TurretValues.convertRotationsToDegrees(encoder.get());
    }

    private boolean atTarget() {
        return Math.abs(getDegrees() - ShooterCalculations.visionYaw) < 0.75;
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
            turretTargetDeg = safeActual + 3;
        } else {
            turretTargetDeg = safeTarget;
        }
    }

    private void setVisionMode(boolean visionMode){
        isVision = visionMode;
    }

    private double getFF() {
        boolean isFF = Math.abs(getDegrees() - turretTargetDeg) > 5;
        if ((getDegrees() - turretTargetDeg) > 0 && isFF) return TurretValues.TurretClockwisekFF;
        else if ((getDegrees() - turretTargetDeg) > 0 && isFF) return -TurretValues.TurretClockwisekFF;
        else return 0;
    }

    private void runPID() {
        handleSafety(isVision);
        motor.set(PID.calculate(getDegrees(), turretTargetDeg) + getFF());
    }

    public void stop() {
        motor.set(0);
    }

    @Override
    public String getDashboardTabName() {
        return "Turret";
    }

    public void setNeutralMode(NeutralMode mode) {
        motor.setNeutralMode(mode);
    }

    public void setForward(boolean isVision) {
        setTurretTargetDegrees(TurretValues.TurretCenterVisionPosition, isVision);
    }

    public void setIntake() {
        setTurretTargetDegrees(Constants.TurretValues.IntakeOrientationDegrees, false);
    }
}


