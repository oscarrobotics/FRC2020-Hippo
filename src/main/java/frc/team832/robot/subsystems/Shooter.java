package frc.team832.robot.subsystems;

import com.revrobotics.CANPIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.control.REVSmartServo_Continuous;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.driverstation.dashboard.DashboardWidget;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.impl.SmartMCAttachedPDPSlot;
import frc.team832.lib.sensors.REVThroughBorePWM;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;
import frc.team832.robot.Constants.ShooterValues;
import frc.team832.robot.utilities.state.ShooterCalculations;

public class Shooter extends SubsystemBase implements DashboardUpdatable {

    public final boolean initSuccessful;

    public boolean isVision = false;

    private final Vision vision;
    private final CANSparkMax primaryMotor, secondaryMotor, turretMotor, feedMotor;
    private final REVSmartServo_Continuous hoodServo;

    private final NetworkTableEntry dashboard_wheelRPM, dashboard_flywheelFF, dashboard_hoodPos, dashboard_turretPos, dashboard_turretPow, dashboard_wheelTargetRPM,
            dashboard_feedWheelRPM, dashboard_feedWheelTargetRPM, dashboard_feedFF, dashboard_turretTarget, dashboard_turretVisionPow, dashboard_isTurretVisionSafe;

    private ShootMode mode = ShootMode.Shooting, lastMode = ShootMode.Shooting;

    private final AnalogInput potentiometer = new AnalogInput(ShooterValues.HOOD_POTENTIOMETER_ANALOG_CHANNEL);
    private final REVThroughBorePWM turretEncoder = new REVThroughBorePWM(ShooterValues.TURRET_ENCODER_DIO_CHANNEL);

    private final PIDController hoodPID = new PIDController(ShooterValues.HoodkP, 0, 0);
    private final PIDController feedPID = new PIDController(ShooterValues.FeedkP, 0, 0);
    private final PIDController turretPID = new PIDController(ShooterValues.TurretkP, 0.0, 0.0);

    private final SmartMCAttachedPDPSlot primaryFlywheelSlot, secondaryFlywheelSlot, turretSlot, feederSlot;

    private double turretPosTarget, feedTarget;

    public Shooter(GrouchPDP pdp, Vision vision) {
        DashboardManager.addTab(this, this);

        primaryMotor = new CANSparkMax(ShooterValues.PRIMARY_CAN_ID, Motor.kNEO);
        secondaryMotor = new CANSparkMax(ShooterValues.SECONDARY_CAN_ID, Motor.kNEO);
        turretMotor = new CANSparkMax(ShooterValues.TURRET_CAN_ID, Motor.kNEO550);
        feedMotor = new CANSparkMax(ShooterValues.FEED_MOTOR_CAN_ID, Motor.kNEO);

        primaryFlywheelSlot = pdp.addDevice(ShooterValues.PRIMARY_PDP_SLOT, primaryMotor);
        secondaryFlywheelSlot = pdp.addDevice(ShooterValues.SECONDARY_PDP_SLOT, secondaryMotor);
        turretSlot = pdp.addDevice(ShooterValues.TURRET_PDP_SLOT, turretMotor);
        feederSlot = pdp.addDevice(ShooterValues.FEEDER_PDP_SLOT, feedMotor);

        hoodServo = new REVSmartServo_Continuous(ShooterValues.HOOD_SERVO_PWM_CHANNEL);

        primaryMotor.wipeSettings();
        secondaryMotor.wipeSettings();
        turretMotor.wipeSettings();
        feedMotor.wipeSettings();

        setFlyheelNeutralMode(NeutralMode.kCoast);
        setTurretNeutralMode(NeutralMode.kBrake);

        primaryMotor.setInverted(false);

        secondaryMotor.follow(primaryMotor, true);

        setCurrentLimit(40);

//        turretPID.reset(getTurretRotations());
        turretPID.reset();
        hoodPID.reset();
        feedPID.reset();

        turretPosTarget = getTurretRotations();

        dashboard_wheelRPM = DashboardManager.addTabItem(this, "Flywheel/RPM", 0.0);
        dashboard_wheelTargetRPM = DashboardManager.addTabItem(this, "Flywheel/Target RPM", 0.0);
        dashboard_flywheelFF = DashboardManager.addTabItem(this, "Flywheel/FF", 0.0);
        dashboard_feedWheelRPM = DashboardManager.addTabItem(this, "Feeder/RPM", 0.0);
        dashboard_feedWheelTargetRPM = DashboardManager.addTabItem(this, "Feeder/Target RPM", 0.0);
        dashboard_feedFF = DashboardManager.addTabItem(this, "Feeder/FF", 0.0);
        dashboard_turretPos = DashboardManager.addTabItem(this, "Turret/Position", 0.0);
        dashboard_turretPow = DashboardManager.addTabItem(this, "Turret/Power", 0.0);
        dashboard_turretTarget = DashboardManager.addTabItem(this, "Turret/Target", 0.0);
        dashboard_hoodPos = DashboardManager.addTabItem(this, "Hood/Position", 0.0);
        dashboard_turretVisionPow = DashboardManager.addTabItem(this, "Turret/Vision Power", 0.0);
        dashboard_isTurretVisionSafe = DashboardManager.addTabItem(this, "Turret/Vision Safe", false, DashboardWidget.BooleanBox);


        this.vision = vision;

        initSuccessful = primaryMotor.getCANConnection() && secondaryMotor.getCANConnection() && turretMotor.getCANConnection() && feedMotor.getCANConnection();
    }

    @Override
    public void periodic() {
        handlePID();
    }


//    final double startPos = getTurretRotations();
//
//
//    private void find_kS() {
//        double difference = Math.abs(startPos - getTurretRotations());
//        System.out.println("Diff:" + difference);
//        if (difference > 0.005) {
//            System.out.println(turretMotor.get());
//            return;
//        }
//
//        double newPower = turretMotor.get() + 0.005;
//        turretMotor.set(newPower);
//
//    }

    @Override
    public void updateDashboardData() {
        dashboard_wheelRPM.setDouble(primaryMotor.getSensorVelocity() * ShooterValues.FlywheelReduction);
        dashboard_feedWheelRPM.setDouble(feedMotor.getSensorVelocity());
        dashboard_turretPos.setDouble(getTurretRotations());
        dashboard_turretPow.setDouble(turretMotor.getOutputVoltage());
        dashboard_turretTarget.setDouble(turretPosTarget);
        dashboard_hoodPos.setDouble(potentiometer.getVoltage());
        dashboard_isTurretVisionSafe.setBoolean(isTurretVisionPosSafe());
        dashboard_turretVisionPow.setDouble(-turretPID.calculate(ShooterCalculations.visionRotation, 0));
    }

    private void setWheelRPM(double wheelTargetRPM) {
        double motorTargetRpm = wheelTargetRPM / ShooterValues.FlywheelReduction;

        double batteryVoltage = primaryMotor.getInputVoltage();
        double ff = ShooterValues.FlywheelFF.calculate(motorTargetRpm) / batteryVoltage;

        if (wheelTargetRPM < primaryMotor.getSensorVelocity() - 50 && mode == ShootMode.Idle) motorTargetRpm = 0;

        dashboard_flywheelFF.setDouble(ff);
        dashboard_wheelTargetRPM.setDouble(wheelTargetRPM * ShooterValues.FlywheelReduction);

        primaryMotor.setTargetVelocity(motorTargetRpm, ShooterValues.FlywheelFF.calculate(motorTargetRpm), CANPIDController.ArbFFUnits.kVoltage);
    }

    public void setDumbFeedRPM(double rpm) {
        setFeedTargetRPM(rpm);
    }

    public void setDumbRPM(double rpm) {
        if (rpm < primaryMotor.getSensorVelocity() - 50)
            setMode(ShootMode.Idle);
        else
            setMode(ShootMode.Shooting);

        setWheelRPM(rpm);
    }

    public void spinUp() {
        setMode(ShootMode.SpinUp);
    }

    public void idle() {
        setMode(ShootMode.Idle);
//        setRPM(2000);
        feedMotor.set(0);
    }

    public void setExitAngle(double degrees) {
        hoodServo.setSpeed(hoodPID.calculate(getHoodAngle(), degrees));
    }

    private void setFeedTargetRPM(double rpm) { feedTarget = rpm; }

    public void setTurretTargetRotation(double pos){
        isVision = false;
        turretPosTarget = pos;
    }

    public void holdTurretPosition() {
        turretPosTarget = getTurretRotations();
    }

    public void setHood(double pow) {
        hoodServo.setSpeed(pow);
    }

    public void spin() {
        turretMotor.set(0.5);
    }

    public void otherSpin() {
        turretMotor.set(-0.5);
    }

    public void stopFeed() {
        feedMotor.set(0);
    }

    public void idleHood() {
        hoodServo.setSpeed(0);
    }

    public void idleTurret() { setTurretTargetRotation(getTurretRotations()); }

    private double getHoodAngle() {
        return OscarMath.map(potentiometer.getVoltage(), 0, 5, 10, 80);
    }

    public double getTurretRotations() { return turretEncoder.get(); }

    public void flywheelTrackTarget() {
        setWheelRPM(ShooterCalculations.flywheelRPM);
    }

    private void hoodTrackTarget() {
        setExitAngle(ShooterCalculations.exitAngle);
    }


    public boolean readyToShoot() {
        return atShootingRpm() && atTurretTarget() && atHoodTarget() && atFeedRpm();
    }

    private boolean atShootingRpm() {
        return Math.abs(primaryMotor.getSensorVelocity() - ShooterCalculations.flywheelRPM) < 100;
    }

    public boolean atFeedRpm() {
        return Math.abs(feedMotor.getSensorVelocity() - ShooterValues.FeedRpm) < 100;
    }

    private boolean atTurretTarget() {
        return Math.abs(getTurretRotations() - ShooterCalculations.visionRotation) < 0.05;
    }

    private boolean atHoodTarget() {
        return Math.abs(getHoodAngle() - ShooterCalculations.exitAngle) < 2;
    }

    public void stopShooter() {
        primaryMotor.set(0);
    }

    public void stopAll() {
        primaryMotor.set(0);
        secondaryMotor.set(0);
        turretMotor.set(0);
        feedMotor.set(0);
    }

    public void trackTarget() {
        isVision = true;
//        flywheelTrackTarget();
        hoodTrackTarget();
    }

    public void setCurrentLimit(int limit) {
        primaryMotor.limitInputCurrent(limit);
        secondaryMotor.limitInputCurrent(limit);
        feedMotor.limitInputCurrent(limit);
    }

    public void setFlyheelNeutralMode(NeutralMode mode) {
        primaryMotor.setNeutralMode(mode);
        secondaryMotor.setNeutralMode(mode);
    }

    public void setTurretNeutralMode(NeutralMode mode) {
        feedMotor.setNeutralMode(mode);
        turretMotor.setNeutralMode(mode);
    }

    private void updatePIDMode() {
        switch (mode) {
            case SpinUp:
                primaryMotor.setPIDF(ShooterValues.SpinupConfig);
                break;
            case Shooting:
                primaryMotor.setPIDF(ShooterValues.ShootingConfig);
                break;
            case Idle:
                primaryMotor.setPIDF(ShooterValues.IdleConfig);
        }
    }

    public void setMode(ShootMode mode) {
        if (this.mode != mode) {
            this.lastMode = this.mode;
            this.mode = mode;
            updatePIDMode();
        }
    }

    private void handlePID() {
        runFeederPID();
        runTurretPID();
    }

    protected TurretSafetyState isTurretSafe(double turretTarget, double turretPosition) {
        if (turretTarget > ShooterValues.PracticeTurretRightPosition || turretPosition > ShooterValues.PracticeTurretRightPosition) return TurretSafetyState.FarRight;
        else if (turretTarget < ShooterValues.PracticeTurretLeftPosition || turretPosition < ShooterValues.PracticeTurretLeftPosition) return TurretSafetyState.FarLeft;
        else return TurretSafetyState.Safe;
    }

    public boolean isTurretVisionPosSafe() {
        return !(getTurretRotations() > ShooterValues.PracticeTurretRightVisionPosition && Math.signum(turretPID.calculate(-ShooterCalculations.visionRotation, 0)) == -1)
                || (getTurretRotations() < ShooterValues.PracticeTurretLeftVisionPosition && Math.signum(turretPID.calculate(-ShooterCalculations.visionRotation, 0)) == 1);
    }

    private void runTurretPID() {
        double power = 0;

       if (isVision) {
           if (vision.getIsValidEntry().getBoolean(false)) {
               double targetOffset;
               if (isTurretVisionPosSafe()) {
                   targetOffset = ShooterCalculations.visionRotation;
               } else {
                   targetOffset = 0.0;
               }
               power = turretPID.calculate(-targetOffset, 0);
           } else {
               power = 0;
           }
       } else {
           double targetRotations = turretPosTarget;
           TurretSafetyState safety = isTurretSafe(targetRotations, getTurretRotations());
           if (safety == TurretSafetyState.FarRight) {
               targetRotations = ShooterValues.PracticeTurretRightPosition;
           } else if (safety == TurretSafetyState.FarLeft) {
               targetRotations = ShooterValues.PracticeTurretLeftPosition;
           }
           power = turretPID.calculate(getTurretRotations(), targetRotations);
       }
       dashboard_turretVisionPow.setDouble(power);
       setTurretPower(power);
    }

    private void runFeederPID() {
        double rpm = feedTarget;
//        double ff = (rpm / ShooterValues.FeedReduction * (Motor.kNEO.freeSpeed / Motor.kNEO.kv)) / 1.8;//Constants.ShooterValues.FEEDER_FF.calculate(rpm)
        double pid = feedPID.calculate(feedMotor.getSensorVelocity(), rpm);

        dashboard_wheelTargetRPM.setDouble(rpm);

        feedMotor.set(pid);
    }

    @Override
    public String getDashboardTabName() {
        return "Shooter";
    }

    public void setTurretPower(double pow) {
        turretMotor.set(pow);
    }

    public enum ShootMode {
        SpinUp,
        Shooting,
        Idle
    }

    public enum TurretSafetyState {
        FarLeft,
        FarRight,
        Safe
    }
}
