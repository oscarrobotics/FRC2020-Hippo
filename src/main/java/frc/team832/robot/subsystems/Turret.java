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
        motor = new CANSparkMax(TurretValues.TURRET_MOTOR_CAN_ID, Motor.kNEO550);
        encoder = new REVThroughBorePWM(TurretValues.TURRET_ENCODER_DIO_CHANNEL);
        pdpSlot = pdp.addDevice(TurretValues.TURRET_PDP_SLOT, motor);

        motor.wipeSettings();
        PID.reset();

        motor.limitInputCurrent(25);
        motor.setNeutralMode(NeutralMode.kBrake);


        // keep turret at init position
        turretTargetDeg = getRotations();

        dashboard_turretPos = DashboardManager.addTabItem(this, "Turret/Position", 0.0);
        dashboard_turretPow = DashboardManager.addTabItem(this, "Turret/Power", 0.0);
        dashboard_turretTarget = DashboardManager.addTabItem(this, "Turret/Target", 0.0);

        initSuccessful = motor.getCANConnection();
    }

    @Override
    public void periodic() {
        runPID();
    }

    @Override
    public void updateDashboardData() {
        dashboard_turretPos.setDouble(getRotations());
        dashboard_turretPow.setDouble(motor.getOutputVoltage());
        dashboard_turretTarget.setDouble(turretTargetDeg);
    }

    public void setTurretTargetDegrees(double pos, boolean isVision) {
        this.isVision = isVision;
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
        return Math.abs(getDegrees() - ShooterCalculations.visionRotation) < 2;
    }

    private void handleSafety(boolean isVisionMode) {
        double rightBound = isVisionMode ? TurretValues.PracticeTurretRightVisionPosition : TurretValues.PracticeTurretRightPosition;
        double leftBound = isVisionMode ? TurretValues.PracticeTurretLeftVisionPosition : TurretValues.PracticeTurretLeftPosition;

        boolean isRightUnsafe = getDegrees() > rightBound || turretTargetDeg > rightBound;
        boolean isLeftUnsafe = getDegrees() < leftBound || turretTargetDeg < leftBound;

        if (isRightUnsafe) {
            if (turretTargetDeg > rightBound) {
                turretTargetDeg = rightBound;
            } else if (getDegrees() > rightBound) {
                turretTargetDeg = rightBound - 5;
            }
        } else if (isLeftUnsafe) {
            if (turretTargetDeg < leftBound) {
                turretTargetDeg = leftBound;
            } else if (getDegrees() < leftBound) {
                turretTargetDeg = leftBound + 5;
            }
        }
    }

    private void runPID() {
        motor.set(PID.calculate(getDegrees(), turretTargetDeg));
        handleSafety(isVision);
    }

    public void stop() {
        motor.set(0);
    }

    @Override
    public String getDashboardTabName() {
        return "Turret";
    }
}


