package frc.team832.robot.subsystems;

import com.revrobotics.CANPIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.controller.PIDController;
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
import frc.team832.robot.Constants.ShooterValues;
import frc.team832.robot.utilities.state.ShooterCalculations;

public class Shooter extends SubsystemBase implements DashboardUpdatable {

    public final boolean initSuccessful;

    public boolean isVision = false;

    private final Vision vision;
    private final CANSparkMax primaryMotor, secondaryMotor, feederMotor;
    private final REVSmartServo_Continuous hoodServo;

    private final NetworkTableEntry dashboard_wheelRPM, dashboard_flywheelFF, dashboard_hoodPos, dashboard_wheelTargetRPM,
            dashboard_feedWheelRPM, dashboard_feedWheelTargetRPM, dashboard_feedFF;

    private ShootMode mode = ShootMode.Shooting, lastMode = ShootMode.Shooting;

    private final AnalogInput potentiometer = new AnalogInput(ShooterValues.HOOD_POTENTIOMETER_ANALOG_CHANNEL);

    private final PIDController hoodPID = new PIDController(ShooterValues.HoodkP, 0, 0);
    private final PIDController feedPID = new PIDController(ShooterValues.FeedkP, 0, 0);

    private final SmartMCAttachedPDPSlot primaryFlywheelSlot, secondaryFlywheelSlot, feederSlot;

    private double feedTarget;

    public Shooter(GrouchPDP pdp, Vision vision) {
        DashboardManager.addTab(this, this);
        this.vision = vision;

        primaryMotor = new CANSparkMax(ShooterValues.PRIMARY_CAN_ID, Motor.kNEO);
        secondaryMotor = new CANSparkMax(ShooterValues.SECONDARY_CAN_ID, Motor.kNEO);
        feederMotor = new CANSparkMax(ShooterValues.FEED_MOTOR_CAN_ID, Motor.kNEO);

        primaryFlywheelSlot = pdp.addDevice(ShooterValues.PRIMARY_PDP_SLOT, primaryMotor);
        secondaryFlywheelSlot = pdp.addDevice(ShooterValues.SECONDARY_PDP_SLOT, secondaryMotor);
        feederSlot = pdp.addDevice(ShooterValues.FEEDER_PDP_SLOT, feederMotor);

        hoodServo = new REVSmartServo_Continuous(ShooterValues.HOOD_SERVO_PWM_CHANNEL);

        primaryMotor.wipeSettings();
        secondaryMotor.wipeSettings();
        feederMotor.wipeSettings();

        setFlyheelNeutralMode(NeutralMode.kCoast);
        setFeederNeutralMode(NeutralMode.kBrake);

        primaryMotor.setInverted(false);

        secondaryMotor.follow(primaryMotor, true);

        primaryMotor.limitInputCurrent(55);
        secondaryMotor.limitInputCurrent(55);
        feederMotor.limitInputCurrent(25);

        hoodPID.reset();
        feedPID.reset();

        // dashboard
        dashboard_wheelRPM = DashboardManager.addTabItem(this, "Flywheel/RPM", 0.0);
        dashboard_wheelTargetRPM = DashboardManager.addTabItem(this, "Flywheel/Target RPM", 0.0);
        dashboard_flywheelFF = DashboardManager.addTabItem(this, "Flywheel/FF", 0.0);
        dashboard_feedWheelRPM = DashboardManager.addTabItem(this, "Feeder/RPM", 0.0);
        dashboard_feedWheelTargetRPM = DashboardManager.addTabItem(this, "Feeder/Target RPM", 0.0);
        dashboard_feedFF = DashboardManager.addTabItem(this, "Feeder/FF", 0.0);
        dashboard_hoodPos = DashboardManager.addTabItem(this, "Hood/Position", 0.0);

        initSuccessful = primaryMotor.getCANConnection() && secondaryMotor.getCANConnection() && feederMotor.getCANConnection();
    }

    @Override
    public void periodic() {
        handlePID();
    }

    // good
    @Override
    public void updateDashboardData() {
        dashboard_wheelRPM.setDouble(primaryMotor.getSensorVelocity() * ShooterValues.FlywheelReduction);
        dashboard_feedWheelRPM.setDouble(feederMotor.getSensorVelocity());
        dashboard_hoodPos.setDouble(potentiometer.getVoltage());
    }

    // good
    private void setWheelRPM(double wheelTargetRPM) {
        double motorTargetRpm = wheelTargetRPM / ShooterValues.FlywheelReduction;

        double batteryVoltage = primaryMotor.getInputVoltage();
        double ff = ShooterValues.FlywheelFF.calculate(motorTargetRpm) / batteryVoltage;

        if (wheelTargetRPM < primaryMotor.getSensorVelocity() - 50 && mode == ShootMode.Idle) motorTargetRpm = 0;

        dashboard_flywheelFF.setDouble(ff);
        dashboard_wheelTargetRPM.setDouble(wheelTargetRPM * ShooterValues.FlywheelReduction);

        primaryMotor.setTargetVelocity(motorTargetRpm, ShooterValues.FlywheelFF.calculate(motorTargetRpm), CANPIDController.ArbFFUnits.kVoltage);
    }

    // testing
    public void setDumbRPM(double rpm) {
        if (rpm < primaryMotor.getSensorVelocity() - 50)
            setMode(ShootMode.Idle);
        else
            setMode(ShootMode.Shooting);

        setWheelRPM(rpm);
    }

    // nuke - handleMode()?
    public void idle() {
        setMode(ShootMode.Idle);
//        setRPM(2000);
        feederMotor.set(0);
    }

    // good
    public void setExitAngle(double degrees) {
        hoodServo.setSpeed(hoodPID.calculate(getHoodAngle(), degrees));
    }

    public void setHood(double pow) {
        hoodServo.setSpeed(pow);
    }

    public void idleHood() {
        hoodServo.setSpeed(0);
    }

    private double getHoodAngle() {
        return OscarMath.map(potentiometer.getVoltage(), 0, 5, 10, 80);
    }

    private void hoodTrackTarget() {
        setExitAngle(ShooterCalculations.exitAngle);
    }

    //superstructure
    boolean readyToShoot() {
        return atShootingRpm() && atHoodTarget() && atFeedRpm();
    }

    private boolean atShootingRpm() {
        return Math.abs(primaryMotor.getSensorVelocity() - ShooterCalculations.flywheelRPM) < 100;
    }

    private boolean atFeedRpm() {
        return Math.abs(feederMotor.getSensorVelocity() - ShooterValues.FeedRpm) < 100;
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
        feederMotor.set(0);
    }

    // external set
    public void setFlyheelNeutralMode(NeutralMode mode) {
        primaryMotor.setNeutralMode(mode);
        secondaryMotor.setNeutralMode(mode);
    }

    // external set
    public void setFeederNeutralMode(NeutralMode mode) {
        feederMotor.setNeutralMode(mode);
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
    }



    private void runFeederPID() {
        double rpm = feedTarget;
//        double ff = (rpm / ShooterValues.FeedReduction * (Motor.kNEO.freeSpeed / Motor.kNEO.kv)) / 1.8;//Constants.ShooterValues.FEEDER_FF.calculate(rpm)
        double pid = feedPID.calculate(feederMotor.getSensorVelocity(), rpm);

        dashboard_wheelTargetRPM.setDouble(rpm);

        feederMotor.set(pid);
    }

    @Override
    public String getDashboardTabName() {
        return "Shooter";
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
