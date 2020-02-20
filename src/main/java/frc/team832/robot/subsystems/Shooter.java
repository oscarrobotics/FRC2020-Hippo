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
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.impl.SmartMCAttachedPDPSlot;
import frc.team832.lib.sensors.REVThroughBorePWM;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants.ShooterValues;

public class Shooter extends SubsystemBase implements DashboardUpdatable {

    public final boolean initSuccessful;


    private final Vision vision;
    private final CANSparkMax primaryMotor, secondaryMotor, turretMotor, feedMotor; //if needed add hood motor
    private final NetworkTableEntry dashboard_wheelRPM, dashboard_PID, dashboard_flywheelFF, dashboard_hoodPos, dashboard_turretPos, dashboard_wheelTargetRPM,
            dashboard_feedWheelRPM, dashboard_feedWheelTargetRPM, dashboard_feedFF;
    private final REVSmartServo_Continuous hoodServo;

    private ShootMode mode = ShootMode.Shooting, lastMode = ShootMode.Shooting;

    private final AnalogInput potentiometer = new AnalogInput(ShooterValues.HOOD_POTENTIOMETER_ANALOG_CHANNEL);
    private final REVThroughBorePWM turretEncoder = new REVThroughBorePWM(ShooterValues.TURRET_ENCODER_DIO_CHANNEL);

    private final PIDController hoodPID = new PIDController(ShooterValues.HoodkP, 0, 0);
    private final PIDController feedPID = new PIDController(ShooterValues.FeedkP, 0, 0);
    private final ProfiledPIDController turretPID = new ProfiledPIDController(ShooterValues.TurretkP, 0, 0, ShooterValues.TurretConstraints);

    private final SmartMCAttachedPDPSlot primaryFlywheelSlot, secondaryFlywheelSlot, turretSlot, feederSlot;

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

        dashboard_wheelRPM = DashboardManager.addTabItem(this, "Flywheel RPM", 0.0);
        dashboard_wheelTargetRPM = DashboardManager.addTabItem(this, "Target Flywheel RPM", 0.0);
        dashboard_PID = DashboardManager.addTabItem(this, "PID", 0.0);
        dashboard_hoodPos = DashboardManager.addTabItem(this, "Hood Position", 0.0);
        dashboard_turretPos = DashboardManager.addTabItem(this, "Turret Position", 0.0);
        dashboard_flywheelFF = DashboardManager.addTabItem(this, "Flywheel FF", 0.0);
        dashboard_feedWheelRPM = DashboardManager.addTabItem(this, "Feed Wheel RPM", 0.0);
        dashboard_feedWheelTargetRPM = DashboardManager.addTabItem(this, "Target Feed Wheel RPM", 0.0);
        dashboard_feedFF = DashboardManager.addTabItem(this, "Feeder FF", 0.0);

        this.vision = vision;

        initSuccessful = primaryMotor.getCANConnection() && secondaryMotor.getCANConnection() && turretMotor.getCANConnection() && feedMotor.getCANConnection();
    }

    public void setFlyheelNeutralMode(NeutralMode mode) {
        primaryMotor.setNeutralMode(mode);
        secondaryMotor.setNeutralMode(mode);
    }

    public void setTurretNeutralMode(NeutralMode mode) {
        feedMotor.setNeutralMode(mode);
        turretMotor.setNeutralMode(mode);
    }

    @Override
    public String getDashboardTabName() {
        return "Shooter";
    }

    @Override
    public void periodic() {
//        trackTarget();
    }

    @Override
    public void updateDashboardData() {
        dashboard_wheelRPM.setDouble(primaryMotor.getSensorVelocity() * ShooterValues.FlywheelReduction);
        dashboard_feedWheelRPM.setDouble(feedMotor.getSensorVelocity());
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

    public void spinUp() {
        setMode(ShootMode.SpinUp);
    }

    public void idle() {
        setMode(ShootMode.Idle);
//        setRPM(2000);
        feedMotor.set(0);
    }

    public void trackTarget() {
        flywheelTrackTarget();
        hoodTrackTarget();
        turretTrackTarget();
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

    public void setPower(double pow) {
        primaryMotor.set(pow);
    }

    public void flywheelTrackTarget() {
        setWheelRPM(vision.calculations.flywheelRPM);
    }

    private void hoodTrackTarget() {
        setExitAngle(vision.calculations.exitAngle);
    }

    private void turretTrackTarget() {
        setHeadingRotation(vision.calculations.turretRotation);
    }

    public boolean readyToShoot() {
        return atShootingRpm() && atTurretTarget() && atHoodTarget() && atFeedRpm();
    }

    private boolean atShootingRpm() {
        return Math.abs(primaryMotor.getSensorVelocity() - vision.calculations.flywheelRPM) < 100;
    }

    private boolean atTurretTarget() {
//        return Math.abs(turretEncoder.getRotations() - vision.getCalculations().turretRotation) < 0.05;
        return false;
    }

    private boolean atHoodTarget() {
        return Math.abs(getHoodAngle() - vision.calculations.exitAngle) < 1;
    }

    public void setHeadingRotation(double rotations) {
        double targetRotation = OscarMath.clip(rotations, 0.25, 0.75);
//        turretMotor.set(turretPID.calculate(turretEncoder.getRotations(), TurretPowerTrain.calculateTicksFromPosition(targetRotation)));
    }

    public void setHeadingDegrees(double degrees) {
        double rotations = OscarMath.clipMap(degrees, -90, 90, 0.25, 0.75);
        setHeadingRotation(rotations);
    }

    public void setExitAngle(double degrees) {
        hoodServo.setSpeed(hoodPID.calculate(getHoodAngle(), degrees));
    }

    private double getHoodAngle() {
        return OscarMath.map(potentiometer.getVoltage(), 0, 5, 10, 80);
    }

    public void spin() {
        turretMotor.set(0.5);
    }

    public void otherSpin() {
        turretMotor.set(-0.5);
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

    public void setCurrentLimit(int limit) {
        primaryMotor.limitInputCurrent(limit);
        secondaryMotor.limitInputCurrent(limit);
        feedMotor.limitInputCurrent(limit);
    }

    public void stopFeed() {
        feedMotor.set(0);
    }

    public void feed(double pow) {
        feedMotor.set(pow);
    }

    public void setDumbFeedRPM(double rpm) {
        setFeedRPM(rpm);
    }

    private void setFeedRPM(double rpm) {
        double ff = (rpm / ShooterValues.FeedReduction * (Motor.kNEO.freeSpeed / Motor.kNEO.kv)) / 1.8;//Constants.ShooterValues.FEEDER_FF.calculate(rpm)
        double pid = feedPID.calculate(feedMotor.getSensorVelocity(), rpm);

        dashboard_feedFF.setDouble(ff);
        dashboard_wheelTargetRPM.setDouble(rpm);

        feedMotor.set(pid + ff);
    }

    public boolean atFeedRpm() {
        return Math.abs(feedMotor.getSensorVelocity() - ShooterValues.FeedRpm) < 100;
    }

    public double getTurretRotations() {
//        return turretEncoder.getRotations();
        return 0;
    }

    public void setDumbRPM(double rpm) {
        if (rpm < primaryMotor.getSensorVelocity() - 50)
            setMode(ShootMode.Idle);
        else
            setMode(ShootMode.Shooting);

        setWheelRPM(rpm);
    }

    public void setHood(double pow) {
        hoodServo.setSpeed(pow);
    }

    public void idleHood() {
        hoodServo.setSpeed(0);
    }

    public enum ShootMode {
        SpinUp,
        Shooting,
        Idle
    }
}
