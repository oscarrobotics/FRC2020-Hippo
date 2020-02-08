package frc.team832.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.robot.Constants;
import frc.team832.robot.accesories.ShooterCalculations;

import static frc.team832.robot.Constants.ShooterValues.TurretPowerTrain;
import static frc.team832.robot.Constants.ShooterValues.TurretReduction;

public class Shooter extends SubsystemBase implements DashboardUpdatable {

    private boolean initSuccessful = false;

    private CANSparkMax primaryMotor, secondaryMotor, turretMotor, feedMotor; //if needed add hood motor
    private NetworkTableEntry dashboard_wheelRPM, dashboard_PID, dashboard_hoodPos, dashboard_turretPos;
    private Servo hoodServo;
    private CANDigitalInput turretLimitInput;

    private ShootMode mode = ShootMode.Idle, lastMode = ShootMode.Idle;

    private PIDController flywheelPID = new PIDController(Constants.ShooterValues.IDLE_kP,0, Constants.ShooterValues.IDLE_kD);
    private ProfiledPIDController turretPID = new ProfiledPIDController(Constants.ShooterValues.TURRET_kP, 0, Constants.ShooterValues.TURRET_kD, Constants.ShooterValues.TURRET_CONSTRAINTS);
    private PIDController hoodPID = new PIDController(Constants.ShooterValues.HOOD_kP, 0, Constants.ShooterValues.HOOD_kD);
    private PIDController feedPID = new PIDController(Constants.SpindexerValues.FEED_kP, 0, Constants.SpindexerValues.FEED_kD);

    private ShooterCalculations shooterCalcs = new ShooterCalculations();

    public Shooter(){
        DashboardManager.addTab(this, this);

        primaryMotor = new CANSparkMax(Constants.ShooterValues.PRIMARY_CAN_ID, Motor.kNEO, false);
        secondaryMotor = new CANSparkMax(Constants.ShooterValues.SECONDARY_CAN_ID, Motor.kNEO, false);
        turretMotor = new CANSparkMax(Constants.ShooterValues.TURRET_CAN_ID, Motor.kNEO550);
        feedMotor = new CANSparkMax(Constants.ShooterValues.FEED_MOTOR_CAN_ID, Motor.kNEO, false);

        hoodServo = new Servo(Constants.ShooterValues.HOOD_CHANNEL);

        primaryMotor.wipeSettings();
        secondaryMotor.wipeSettings();
        turretMotor.wipeSettings();
        feedMotor.wipeSettings();

        secondaryMotor.follow(primaryMotor);

        NeutralMode flywheelMode = NeutralMode.kCoast;
        primaryMotor.setNeutralMode(flywheelMode);
        secondaryMotor.setNeutralMode(flywheelMode);

        turretMotor.setNeutralMode(NeutralMode.kBrake);

        feedMotor.setNeutralMode(NeutralMode.kCoast);

        primaryMotor.setInverted(false);
        secondaryMotor.setInverted(false);
        primaryMotor.setSensorPhase(true);
        secondaryMotor.setSensorPhase(true);

        turretMotor.setInverted(false);
        turretMotor.setSensorPhase(true);

        feedMotor.setInverted(false);
        feedMotor.setSensorPhase(true);

        turretLimitInput = new CANDigitalInput(turretMotor.getBaseController(), CANDigitalInput.LimitSwitch.kForward, CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);

        setCurrentLimit(40);

        dashboard_wheelRPM = DashboardManager.addTabItem(this, "RPM", 0.0);
        dashboard_PID = DashboardManager.addTabItem(this, "PID", 0.0);
        dashboard_hoodPos = DashboardManager.addTabItem(this, "Hood Position", 0.0);
        dashboard_turretPos = DashboardManager.addTabItem(this, "Turret Position", 0.0);

        initSuccessful = true;
    }

    @Override
    public String getDashboardTabName () {
        return "Shooter";
    }

    @Override
    public void periodic() {
        updatePIDMode();
        shooterCalcs.update();
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
                flywheelPID.setPID(Constants.ShooterValues.SPIN_UP_kP, 0, Constants.ShooterValues.SPIN_UP_kD);
                break;
            case Shooting:
                flywheelPID.setPID(Constants.ShooterValues.SHOOTING_kP, 0, Constants.ShooterValues.SHOOTING_kD);
                break;
            case SpinDown:
                flywheelPID.setPID(Constants.ShooterValues.SPIN_DOWN_kP, 0, Constants.ShooterValues.SPIN_DOWN_kD);
                break;
            case Idle:
                flywheelPID.setPID(Constants.ShooterValues.IDLE_kP, 0, Constants.ShooterValues.IDLE_kD);
                break;
        }
    }

    public void setMode(ShootMode mode) {
        this.lastMode = this.mode;
        this.mode = mode;
    }

    public void spinUp() {
        setMode(ShootMode.SpinUp);
    }

    public void idle() {
        setMode(ShootMode.Idle);
    }

    public void trackTarget() {
        flywheelTrackTarget();
        hoodTrackTarget();
        turretTrackTarget();
    }

    public void flywheelTrackTarget() {
        primaryMotor.set(flywheelPID.calculate(primaryMotor.getSensorVelocity(), shooterCalcs.flywheelRPM));
    }

    private void setRPM(double rpm) {
        double power = flywheelPID.calculate(primaryMotor.getSensorVelocity(), rpm);
        dashboard_PID.setDouble(power);
        primaryMotor.set(power);
    }

    private void hoodTrackTarget() {
        hoodServo.set(hoodPID.calculate(hoodServo.getPosition(), shooterCalcs.hoodPosition));
    }

    private void turretTrackTarget() {
        turretMotor.set(turretPID.calculate(turretMotor.getSensorPosition(), shooterCalcs.turretPosition));
    }

    public boolean readyToShoot() {
        return atShootingRpm() && atTurretTarget() && atHoodTarget() && atFeedRpm();
    }

    private boolean atShootingRpm() {
        return Math.abs(primaryMotor.getSensorVelocity() - shooterCalcs.flywheelRPM) < 100;
    }

    private boolean atTurretTarget() {
        return Math.abs(turretMotor.getSensorPosition() - shooterCalcs.turretPosition) < 10;
    }

    public void setHeadingRotation(double rotation) {
        double power = turretPID.calculate(turretMotor.getSensorPosition(), TurretPowerTrain.calculateTicksFromPosition(rotation));
        turretMotor.set(power);
    }

    public void
    spin() {
        turretMotor.set(0.6);
    }

    private boolean atHoodTarget() {
        return Math.abs(hoodServo.getPosition() - shooterCalcs.hoodPosition) < 0.05;
    }

    public void stopShooter() {
        primaryMotor.set(0);
    }

    public void stopAll() {
        primaryMotor.set(0);
        secondaryMotor.set(0);
        turretMotor.set(0);
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

    public void setFeedRPM(double rpm) {
        feedPID.calculate(feedMotor.getSensorVelocity(), rpm);
    }

    public boolean atFeedRpm() {
        return Math.abs(feedMotor.getSensorVelocity() - Constants.ShooterValues.FEED_RPM) < 100;
    }

    public double getTurretRotations() {
        return turretMotor.getSensorPosition() * TurretReduction;
    }

    public void stop() {
    }

    public enum ShootMode {
        SpinUp,
        Shooting,
        SpinDown,
        Idle
    }
}
