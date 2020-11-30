package frc.team832.robot.subsystems;

import com.revrobotics.CANPIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.team832.lib.control.REVSmartServo_Continuous;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.impl.SmartMCAttachedPDPSlot;
import frc.team832.lib.sensors.REVThroughBoreRelative;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;
import frc.team832.robot.Constants.ShooterValues;
import frc.team832.robot.utilities.state.ShooterCalculations;

import static frc.team832.robot.Constants.ShooterValues.*;

public class Shooter extends SubsystemBase {

    public final boolean initSuccessful;

    public boolean isVision = false;

    private final CANSparkMax primaryMotor, secondaryMotor, feederMotor;
    private final REVSmartServo_Continuous hoodServo;

    private final REVThroughBoreRelative flywheelEncoder;

    private final NetworkTableEntry dashboard_wheelRPM, dashboard_motorRPM, dashboard_flywheelFF, dashboard_hoodPos, dashboard_hoodAngle, dashboard_wheelTargetRPM,
            dashboard_feedWheelRPM, dashboard_feedWheelTargetRPM, dashboard_feedFF, dashboard_shootMode, dashboard_flywheelPID;

    private final AnalogInput potentiometer = new AnalogInput(ShooterValues.HOOD_POTENTIOMETER_ANALOG_CHANNEL);

    private final PIDController hoodPID = new PIDController(ShooterValues.HoodkP, 0, 0);
    private final PIDController feedPID = new PIDController(ShooterValues.FeedkP, 0, 0);
    private final PIDController flywheelPID = new PIDController(ShootingConfig.getkP(), 0, 0);

    private final SmartMCAttachedPDPSlot primaryFlywheelSlot, secondaryFlywheelSlot, feederSlot;

    private double feedTarget, hoodTarget, flywheelTarget;

    public Shooter(GrouchPDP pdp) {
        setName("Shooter");
        DashboardManager.addTab(this);

        primaryMotor = new CANSparkMax(ShooterValues.PRIMARY_CAN_ID, Motor.kNEO);
        secondaryMotor = new CANSparkMax(ShooterValues.SECONDARY_CAN_ID, Motor.kNEO);
        feederMotor = new CANSparkMax(ShooterValues.FEED_MOTOR_CAN_ID, Motor.kNEO);

        primaryFlywheelSlot = pdp.addDevice(ShooterValues.PRIMARY_PDP_SLOT, primaryMotor);
        secondaryFlywheelSlot = pdp.addDevice(ShooterValues.SECONDARY_PDP_SLOT, secondaryMotor);
        feederSlot = pdp.addDevice(ShooterValues.FEEDER_PDP_SLOT, feederMotor);

        hoodServo = new REVSmartServo_Continuous(ShooterValues.HOOD_SERVO_PWM_CHANNEL);

        flywheelEncoder = new REVThroughBoreRelative(
                ShooterValues.FLYWHEEL_ENCODER_DIO_CHANNEL_A,
                ShooterValues.FLYWHEEL_ENCODER_DIO_CHANNEL_B,
                true,
                CounterBase.EncodingType.k1X
        );

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
        flywheelPID.reset();

        // dashboard
        dashboard_wheelRPM = DashboardManager.addTabItem(this, "Flywheel/WheelRPM", 0.0);
        dashboard_motorRPM = DashboardManager.addTabItem(this, "Flywheel/MotorRPM", 0.0);
        dashboard_wheelTargetRPM = DashboardManager.addTabItem(this, "Flywheel/Target RPM", 0.0);
        dashboard_flywheelFF = DashboardManager.addTabItem(this, "Flywheel/FF", 0.0);
        dashboard_flywheelPID = DashboardManager.addTabItem(this, "Flywheel/PID", 0.0);
        dashboard_feedWheelRPM = DashboardManager.addTabItem(this, "Feeder/RPM", 0.0);
        dashboard_feedWheelTargetRPM = DashboardManager.addTabItem(this, "Feeder/Target RPM", 0.0);
        dashboard_feedFF = DashboardManager.addTabItem(this, "Feeder/FF", 0.0);
        dashboard_hoodPos = DashboardManager.addTabItem(this, "Hood/Position", 0.0);
        dashboard_hoodAngle = DashboardManager.addTabItem(this, "Hood/Angle", 0.0);
        dashboard_shootMode = DashboardManager.addTabItem(this, "Mode", "Unknown");

        DashboardManager.getTab(this).add("FlywheelPID", flywheelPID);
        DashboardManager.getTab(this).add("FeederPID", feedPID);

        initSuccessful = primaryMotor.getCANConnection() && secondaryMotor.getCANConnection() && feederMotor.getCANConnection();
    }

    @Override
    public void periodic() {
        handlePID();
        dashboard_wheelRPM.setDouble(getFlywheelRPM_Encoder());
        dashboard_motorRPM.setDouble(primaryMotor.getSensorVelocity());
        dashboard_wheelTargetRPM.setDouble(flywheelTarget);
        dashboard_feedWheelRPM.setDouble(feederMotor.getSensorVelocity());
        dashboard_hoodPos.setDouble(potentiometer.getVoltage());
        dashboard_hoodAngle.setDouble(getHoodAngle());
    }

    public void setFlywheelRPM(double wheelTargetRPM) {
        flywheelTarget = wheelTargetRPM;
    }

    public void idle() {
        setFeedRPM(0);
        setFlywheelRPM(0);
    }

    public void prepareShoot() {
        setFeedRPM(0);
        setFlywheelRPM(ShooterCalculations.flywheelRPM);//ShooterCalculations.flywheelRPM
        setHoodAngle(ShooterCalculations.exitAngle);
    }

    public void shoot() {
        setFeedRPM(ShooterValues.FeedRpm);
        setFlywheelRPM(ShooterCalculations.flywheelRPM);
        setHoodAngle(ShooterCalculations.exitAngle);
    }

    public void trackTarget(boolean isFiring) {
        if (isFiring) {
            shoot();
        } else {
            prepareShoot();
        }
    }

    public void setFeedRPM(double rpm) {
        feedTarget = rpm;
    }


    public void setHood(double potVoltage) {
        hoodTarget = potVoltage;
    }

    public void idleHood() {
        hoodServo.setSpeed(0);
    }

    public void setHoodAngle(double degrees) {
        setHood(calculateVoltageFromAngle(degrees));
    }

    private double calculateVoltageFromAngle(double degrees) {
        return OscarMath.clipMap(degrees, ShooterValues.HoodMinAngle, ShooterValues.HoodMaxAngle, ShooterValues.HoodBottom, ShooterValues.HoodTop);
    }

    private double getHoodAngle() {
        return OscarMath.map(potentiometer.getVoltage(), ShooterValues.HoodBottom, ShooterValues.HoodTop, ShooterValues.HoodMinAngle, ShooterValues.HoodMaxAngle);
    }

    public boolean readyToShoot() {
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

    public void setFlyheelNeutralMode(NeutralMode mode) {
        primaryMotor.setNeutralMode(mode);
        secondaryMotor.setNeutralMode(mode);
    }

    public void setFeederNeutralMode(NeutralMode mode) {
        feederMotor.setNeutralMode(mode);
    }

    private void handlePID() {
        runFeederPID();
        runHoodPID();
        runFlywheelPID();
    }

    private void runFlywheelPID() {
        if (flywheelTarget == 0) {
            primaryMotor.set(0);
            return;
        }

        double batteryVoltage = primaryMotor.getInputVoltage();
        double ff = (ShooterValues.FlywheelFF.calculate(flywheelTarget) / batteryVoltage) / 2;

        double power = flywheelPID.calculate(getFlywheelRPM_Encoder(), flywheelTarget);

        dashboard_flywheelFF.setDouble(ff);
        dashboard_flywheelPID.setDouble(power);

        primaryMotor.set(power + ff);
    }

    private void runHoodPID() {
        double speed = OscarMath.clip(hoodPID.calculate(potentiometer.getVoltage(), hoodTarget), -0.99, 0.99);
        hoodServo.setSpeed(speed);
    }

    private void runFeederPID() {
        double rpm = feedTarget;
//        double ff = (rpm / ShooterValues.FeedReduction * (Motor.kNEO.freeSpeed / Motor.kNEO.kv)) / 1.8;//Constants.ShooterValues.FEEDER_FF.calculate(rpm)
        double pid = feedPID.calculate(feederMotor.getSensorVelocity(), rpm);

        dashboard_wheelTargetRPM.setDouble(rpm);

        feederMotor.set(pid);
    }

    public double getPotentiometer() {
        return potentiometer.getVoltage();
    }

    public double getFlywheelRPM_Encoder() {
        return (flywheelEncoder.getRate() / 2048) * 60;
    }
}
