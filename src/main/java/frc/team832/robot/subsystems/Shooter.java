package frc.team832.robot.subsystems;

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
import frc.team832.robot.Constants;
import frc.team832.robot.utilities.state.ShooterCalculations;

import static frc.team832.robot.Constants.ShooterValues.TurretPowerTrain;
import static frc.team832.robot.Constants.ShooterValues.TurretReduction;

public class Shooter extends SubsystemBase implements DashboardUpdatable {

    private boolean initSuccessful = false;

    private CANSparkMax primaryMotor, secondaryMotor, turretMotor, feedMotor; //if needed add hood motor
    private NetworkTableEntry dashboard_wheelRPM, dashboard_PID, dashboard_ff, dashboard_hoodPos, dashboard_turretPos, dashboard_wheelTargetRPM;
    private REVSmartServo_Continuous hoodServo;

    private ShootMode mode = ShootMode.Shooting, lastMode = ShootMode.Shooting;

    private AnalogInput potentiometer = new AnalogInput(1);
//    private REVThroughBorePWM turretEncoder = new REVThroughBorePWM(0, 1);

    private PIDController flywheelPID = new PIDController(Constants.ShooterValues.SHOOTING_kP,0, 0);
    private PIDController hoodPID = new PIDController(Constants.ShooterValues.HOOD_kP, 0, 0);
    private PIDController feedPID = new PIDController(Constants.SpindexerValues.FEED_kP, 0, 0);
    private ProfiledPIDController turretPID = new ProfiledPIDController(Constants.ShooterValues.TURRET_kP, 0, 0, Constants.ShooterValues.TURRET_CONSTRAINTS);

    private ShooterCalculations shooterCalcs = new ShooterCalculations();

    private SmartMCAttachedPDPSlot primaryFlywheelSlot, secondaryFlywheelSlot, turretSlot, feederSlot;

    public Shooter(GrouchPDP pdp){
        DashboardManager.addTab(this, this);

        primaryMotor = new CANSparkMax(Constants.ShooterValues.PRIMARY_CAN_ID, Motor.kNEO);
        secondaryMotor = new CANSparkMax(Constants.ShooterValues.SECONDARY_CAN_ID, Motor.kNEO);
        turretMotor = new CANSparkMax(Constants.ShooterValues.TURRET_CAN_ID, Motor.kNEO550);
        feedMotor = new CANSparkMax(Constants.ShooterValues.FEED_MOTOR_CAN_ID, Motor.kNEO);

        primaryFlywheelSlot = pdp.addDevice(Constants.ShooterValues.PRIMARY_PDP_SLOT, primaryMotor);
        secondaryFlywheelSlot = pdp.addDevice(Constants.ShooterValues.SECONDARY_PDP_SLOT, secondaryMotor);
        turretSlot = pdp.addDevice(Constants.ShooterValues.TURRET_PDP_SLOT, turretMotor);
        feederSlot = pdp.addDevice(Constants.ShooterValues.FEEDER_PDP_SLOT, feedMotor);

        hoodServo = new REVSmartServo_Continuous(Constants.ShooterValues.HOOD_SERVO_CHANNEL);

        primaryMotor.wipeSettings();
        secondaryMotor.wipeSettings();
        turretMotor.wipeSettings();
        feedMotor.wipeSettings();

        setFlyheelNeutralMode(NeutralMode.kCoast);
        setTurretMode(NeutralMode.kBrake);

        primaryMotor.setInverted(false);

        secondaryMotor.follow(primaryMotor, true);

        setCurrentLimit(40);

        DashboardManager.getTab(this).add(flywheelPID);

        dashboard_wheelRPM = DashboardManager.addTabItem(this, "RPM", 0.0);
        dashboard_PID = DashboardManager.addTabItem(this, "PID", 0.0);
        dashboard_hoodPos = DashboardManager.addTabItem(this, "Hood Position", 0.0);
        dashboard_turretPos = DashboardManager.addTabItem(this, "Turret Position", 0.0);
        dashboard_wheelTargetRPM = DashboardManager.addTabItem(this, "Target RPM", 0.0);
        dashboard_ff = DashboardManager.addTabItem(this, "FeedForward", 0.0);

        initSuccessful = true;
    }

    public void setFlyheelNeutralMode(NeutralMode mode) {
        primaryMotor.setNeutralMode(mode);
        secondaryMotor.setNeutralMode(mode);
    }

    public void setTurretMode(NeutralMode mode) {
        feedMotor.setNeutralMode(mode);
        turretMotor.setNeutralMode(mode);
    }

    @Override
    public String getDashboardTabName () {
        return "Shooter";
    }

    @Override
    public void periodic() {
//        trackTarget();
    }

    @Override
    public void updateDashboardData () {
        dashboard_wheelRPM.setDouble(primaryMotor.getSensorVelocity());
    }

    public boolean isInitSuccessful() { return initSuccessful; }

    private void updatePIDMode() {
        switch (mode) {
            case SpinUp:
                flywheelPID.setPID(Constants.ShooterValues.SPIN_UP_kP, 0, 0);
                break;
            case Shooting:
                flywheelPID.setPID(Constants.ShooterValues.SHOOTING_kP, 0, 0);
                break;
            case Idle:
                flywheelPID.setPID(Constants.ShooterValues.IDLE_kP, 0, 0);
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
        setRPM(2000);
    }

    public void trackTarget() {
        flywheelTrackTarget();
        hoodTrackTarget();
        turretTrackTarget();
    }

    private void setRPM(double rpm) {
//        double power = primaryMotor.getSensorVelocity() - rpm > 200 ? 0 : OscarMath.clip(flywheelPID.calculate(primaryMotor.getSensorVelocity(), rpm) + Constants.ShooterValues.FLYWHEEL_FF.calculate(rpm), -0.5, 0.5);
//        double ff = Constants.ShooterValues.FLYWHEEL_FF.calculate(rpm);

        double ff = (rpm / (Constants.ShooterValues.FlywheelReduction * (Motor.kNEO.freeSpeed / (Motor.kNEO.kv / 12))) / 12) / 1.8;
        double pid = flywheelPID.calculate(primaryMotor.getSensorVelocity(), rpm);
        double power = OscarMath.clip(ff + pid, -0.6, 0.6);
        dashboard_PID.setDouble(pid);
        dashboard_ff.setDouble(ff);
        primaryMotor.set(power);
    }

    public void setPower(double pow) {
        primaryMotor.set(pow);
    }

    public void flywheelTrackTarget() {
        primaryMotor.set(flywheelPID.calculate(primaryMotor.getSensorVelocity(), shooterCalcs.flywheelRPM));
    }

    private void hoodTrackTarget() {
        setExitAngle(shooterCalcs.exitAngle);
    }

    private void turretTrackTarget() {
        setHeadingRotation(shooterCalcs.turretRotation);
    }

    public boolean readyToShoot() {
        return atShootingRpm() && atTurretTarget() && atHoodTarget() && atFeedRpm();
    }

    private boolean atShootingRpm() {
        return Math.abs(primaryMotor.getSensorVelocity() - shooterCalcs.flywheelRPM) < 100;
    }

    private boolean atTurretTarget() {
//        return Math.abs(turretEncoder.getRotations() - shooterCalcs.turretRotation) < 0.05;
        return false;
    }

    private boolean atHoodTarget() {
        return Math.abs(getHoodAngle() - shooterCalcs.exitAngle) < 1;
    }//this is not right and needs to be looked over

    public void setHeadingRotation(double rotations) {
        double targetRotation = OscarMath.clip(rotations, 0.25, 0.75);
//        turretMotor.set(turretPID.calculate(turretEncoder.getRotations(), TurretPowerTrain.calculateTicksFromPosition(targetRotation)));
    }

    public void setHeadingDegrees(double degrees) {
        double rotations = OscarMath.clipMap(degrees, -90, 90, -0.5, 0.5);
        setHeadingRotation(rotations);
    }

    public void setExitAngle(double degrees) {
        hoodServo.setSpeed(hoodPID.calculate(getHoodAngle(), degrees));
    }

    private double getHoodAngle() {
        return OscarMath.map(potentiometer.getVoltage(), 0, 5, 10, 80);
    }

    public void spin() {
        turretMotor.set(0.6);
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
        feedMotor.set(feedPID.calculate(feedMotor.getSensorVelocity(), rpm) + Constants.ShooterValues.FEEDER_FF.calculate(rpm));
    }

    public boolean atFeedRpm() {
        return Math.abs(feedMotor.getSensorVelocity() - Constants.ShooterValues.FeedRpm) < 100;
    }

    public double getTurretRotations() {
//        return turretEncoder.getRotations();
        return 0;
    }

    public void setDumbRPM(double rpm) {
        setMode(ShootMode.Shooting);
        dashboard_wheelTargetRPM.setDouble(rpm);
        setRPM(rpm);
    }

    public enum ShootMode {
        SpinUp,
        Shooting,
        Idle
    }
}
