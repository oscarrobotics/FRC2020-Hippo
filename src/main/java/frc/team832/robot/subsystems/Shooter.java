package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
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

    public final CANSparkMax primaryMotor;
    private final CANSparkMax secondaryMotor;
    private final CANSparkMax feederMotor;
    private final REVSmartServo_Continuous hoodServo;

    private final REVThroughBoreRelative flywheelEncoder;

    private final NetworkTableEntry dashboard_wheelRPM, dashboard_motorRPM, dashboard_flywheelFFEffort, dashboard_hoodPos, dashboard_hoodAngle, dashboard_flywheelTargetRPM,
            dashboard_feedWheelRPM, dashboard_feedWheelTargetRPM, dashboard_feedFF, dashboard_feedWheelPIDEffort, dashboard_flywheelPIDEffort, dashboard_flywheelStateSpaceEffort,
            dashboard_flywheelAtTarget, dashboard_feederAtTarget, dashboard_hoodAtTarget, dashboard_flywheelError, dashboard_hoodTarget;

    private final AnalogInput potentiometer = new AnalogInput(ShooterValues.HOOD_POTENTIOMETER_ANALOG_CHANNEL);

    private final PIDController hoodPID = new PIDController(ShooterValues.HoodkP, 0, 0);
    private final PIDController feedPID = new PIDController(ShooterValues.FeedkP, 0, 0);
    private final PIDController flywheelPID = new PIDController(ShootingConfig.getkP(), ShootingConfig.getkI(), ShootingConfig.getkD());

    private final KalmanFilter<N1, N1, N1> m_observer = new KalmanFilter<>(
            Nat.N1(), Nat.N1(),
            Constants.ShooterValues.m_flywheelPlant,
            VecBuilder.fill(2.5), // How accurate we think our model is
            VecBuilder.fill(0.0065), // How accurate we think our encoder data is
            ShooterValues.ControlLoopPeriod);

    private final LinearQuadraticRegulator<N1, N1, N1> m_controller
            = new LinearQuadraticRegulator<>(Constants.ShooterValues.m_flywheelPlant,
            VecBuilder.fill(120.0), // qelms. Velocity error tolerance, in radians per second. Decrease
            // this to more heavily penalize state excursion, or make the controller behave more aggressively.
            VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
            // heavily penalize control effort, or make the controller less aggressive. 12 is a good
            // starting point because that is the (approximate) maximum voltage of a battery.
            ShooterValues.ControlLoopPeriod); // Nominal time between loops. 0.020 for TimedRobot, but can be lower if using notifiers.

    private final LinearSystemLoop<N1, N1, N1> flywheelLoop = new LinearSystemLoop<>(
            Constants.ShooterValues.m_flywheelPlant,
            m_controller,
            m_observer,
            12.0,
            ShooterValues.ControlLoopPeriod);

    @SuppressWarnings({"FieldCanBeLocal", "unused"})
    private final SmartMCAttachedPDPSlot primaryFlywheelSlot, secondaryFlywheelSlot, feederSlot;

    private double feedTarget, hoodTarget, flywheelTargetRPM;

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
        flywheelEncoder.setDistancePerPulse(Units.rotationsPerMinuteToRadiansPerSecond(1.0 / 2048 * 60));

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

        flywheelPID.setTolerance(100, 100);
        feedPID.setTolerance(100, 100);

        // dashboard
        dashboard_wheelRPM = DashboardManager.addTabItem(this, "Flywheel/WheelRPM", 0.0);
        dashboard_motorRPM = DashboardManager.addTabItem(this, "Flywheel/MotorRPM", 0.0);
        dashboard_flywheelTargetRPM = DashboardManager.addTabItem(this, "Flywheel/Target RPM", 0.0);
        dashboard_flywheelFFEffort = DashboardManager.addTabItem(this, "Flywheel/FFEffort", 0.0);
        dashboard_flywheelPIDEffort = DashboardManager.addTabItem(this, "Flywheel/PIDEffort", 0.0);
        dashboard_flywheelStateSpaceEffort = DashboardManager.addTabItem(this, "Flywheel/StateSpaceEffort", 0.0);
        dashboard_flywheelAtTarget = DashboardManager.addTabItem(this, "Flywheel/AtTarget", false);
        dashboard_flywheelError = DashboardManager.addTabItem(this, "Flywheel/Error", 0.0);
        dashboard_feedWheelRPM = DashboardManager.addTabItem(this, "Feeder/RPM", 0.0);
        dashboard_feedWheelTargetRPM = DashboardManager.addTabItem(this, "Feeder/Target RPM", 0.0);
        dashboard_feedFF = DashboardManager.addTabItem(this, "Feeder/FFEffort", 0.0);
        dashboard_feedWheelPIDEffort = DashboardManager.addTabItem(this, "Feeder/PIDEffort", 0.0);
        dashboard_feederAtTarget = DashboardManager.addTabItem(this, "Feeder/AtTarget", false);
        dashboard_hoodPos = DashboardManager.addTabItem(this, "Hood/Position", 0.0);
        dashboard_hoodAngle = DashboardManager.addTabItem(this, "Hood/Angle", 0.0);
        dashboard_hoodTarget = DashboardManager.addTabItem(this, "Hood/Target", 0.0);
        dashboard_hoodAtTarget = DashboardManager.addTabItem(this, "Hood/AtTarget", false);

        DashboardManager.getTab(this).add("FlywheelPID", flywheelPID);
        DashboardManager.getTab(this).add("FeederPID", feedPID);

        initSuccessful = primaryMotor.getCANConnection() && secondaryMotor.getCANConnection() && feederMotor.getCANConnection();
    }

    @Override
    public void periodic() {
        dashboard_flywheelTargetRPM.setDouble(flywheelTargetRPM);
        dashboard_flywheelError.setDouble(flywheelTargetRPM - getFlywheelRPM_Encoder());
        dashboard_wheelRPM.setDouble(getFlywheelRPM_Encoder());
        dashboard_motorRPM.setDouble(primaryMotor.getSensorVelocity());
        dashboard_feedWheelRPM.setDouble(feederMotor.getSensorVelocity());
        dashboard_hoodPos.setDouble(potentiometer.getVoltage());
        dashboard_hoodAngle.setDouble(getHoodAngle());
        dashboard_flywheelAtTarget.setBoolean(atShootingRpm());
        dashboard_feederAtTarget.setBoolean(atFeedRpm());
        dashboard_hoodTarget.setDouble(getHoodTargetAngle());
        dashboard_hoodAtTarget.setBoolean(atHoodAngle());
    }

    public void updateControlLoops() {
        double flywheelTargetRadiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(flywheelTargetRPM);
        double flywheelStateSpaceEffortVolts = calculateFlywheelStateSpace(flywheelTargetRadiansPerSecond);

        dashboard_flywheelStateSpaceEffort.setDouble(flywheelStateSpaceEffortVolts / 12);
        setFlywheelVoltage(flywheelStateSpaceEffortVolts);

//        runFlywheelPID();
        runFeederPID();
        runHoodPID();
    }

    private double calculateFlywheelStateSpace(double radsPerSec) {
        if (radsPerSec == 0) {
            return 0;
        }

        flywheelLoop.setNextR(VecBuilder.fill(radsPerSec));

        // Correct our Kalman filter's state vector estimate with encoder data.
        flywheelLoop.correct(VecBuilder.fill(flywheelEncoder.getRate()));

        // Update our LQR to generate new voltage commands and use the voltages to predict the next
        // state with out Kalman filter.
        flywheelLoop.predict(ShooterValues.ControlLoopPeriod);

        // Send the new calculated voltage to the motors.
        // voltage = duty cycle * battery voltage, so
        // duty cycle = voltage / battery voltage
        return flywheelLoop.getU(0);
    }

    public void setFlywheelRPM(double rpm) {
        flywheelTargetRPM = rpm;
    }

    public void trackTarget() {
        setFlywheelRPM(ShooterCalculations.flywheelRPM);
        setHoodAngle(ShooterCalculations.exitAngle);
    }

    public void setHoodToVisionDistance() {
        setHoodAngle(ShooterCalculations.exitAngle);
    }

    public void dumbShoot() {

    }

    public void setFeedRPM(double rpm) {
        feedTarget = rpm;
    }

    private void setHood(double potVoltage) {
        hoodTarget = potVoltage;
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

    public boolean atShootingRpm() {
        return OscarMath.withinEpsilon(300, flywheelTargetRPM, getFlywheelRPM_Encoder());
    }

    public boolean atFeedRpm() {
        return OscarMath.withinEpsilon(500, feedTarget, feederMotor.getSensorVelocity());
    }

    public boolean atHoodAngle() {
        return OscarMath.withinEpsilon(1, getHoodTargetAngle(), getHoodAngle());
    }

    public double getHoodTargetAngle() {
        return OscarMath.map(hoodTarget, HoodTop, HoodBottom, HoodMaxAngle, HoodMinAngle);
    }

    public void setFlyheelNeutralMode(NeutralMode mode) {
        primaryMotor.setNeutralMode(mode);
        secondaryMotor.setNeutralMode(mode);
    }

    public void setFeederNeutralMode(NeutralMode mode) {
        feederMotor.setNeutralMode(mode);
    }

    private void runFlywheelPID() {
        if (flywheelTargetRPM == 0) {
            primaryMotor.set(0);
            dashboard_flywheelFFEffort.setDouble(0);
            dashboard_flywheelPIDEffort.setDouble(0);
            return;
        }

        dashboard_flywheelTargetRPM.setDouble(flywheelTargetRPM);

        double batteryVoltage = primaryMotor.getInputVoltage();
        double ff = (ShooterValues.FlywheelFF.calculate(flywheelTargetRPM) / batteryVoltage) * FlywheelReduction;

        double power = flywheelPID.calculate(getFlywheelRPM_Encoder(), flywheelTargetRPM);

        dashboard_flywheelFFEffort.setDouble(ff);
        dashboard_flywheelPIDEffort.setDouble(power);

        primaryMotor.set(power + ff);
    }

    private void runHoodPID() {
        double speed = OscarMath.clip(hoodPID.calculate(potentiometer.getVoltage(), hoodTarget), -0.99, 0.99);
        hoodServo.setSpeed(speed);
    }

    private void runFeederPID() {
        if(feedTarget == 0) {
            feederMotor.set(0);
            dashboard_feedFF.setDouble(0);
            dashboard_feedWheelTargetRPM.setDouble(0);
            return;
        }
        
        double ffEffort = ((1.0/473.0) * feedTarget) / 12.0;
        dashboard_feedFF.setDouble(ffEffort);

        double pidEffort = feedPID.calculate(feederMotor.getSensorVelocity(), feedTarget);
        dashboard_feedWheelTargetRPM.setDouble(feedTarget);
        dashboard_feedWheelPIDEffort.setDouble(pidEffort);

        feederMotor.set(ffEffort + pidEffort);
    }

    public void setFlywheelVoltage(double volts){
        primaryMotor.set(volts / primaryMotor.getInputVoltage());
    }

    public double getPotentiometer() {
        return potentiometer.getVoltage();
    }

    public double getFlywheelRPM_Encoder() {
        return Units.radiansPerSecondToRotationsPerMinute(flywheelEncoder.getRate());
    }

    public void idleAll() {
        setHoodAngle(getHoodAngle());
        setFeedRPM(0);
        setFlywheelRPM(0);
    }

    public void close() {
        flywheelEncoder.close();
        potentiometer.close();
        hoodServo.close();
    }
}
