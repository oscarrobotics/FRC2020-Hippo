package frc.team832.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.robot.Constants;
import frc.team832.robot.commands.TemplateCommand;

public class Shooter extends SubsystemBase implements DashboardUpdatable {

    private boolean initSuccessful = false;

    private CANSparkMax primaryMotor, secondaryMotor, hoodMotor, turretMotor; //if needed add hood motor
    private NetworkTableEntry dashboard_wheelRPM, dashboard_PID;

    PIDController pid = new PIDController(Constants.ShooterValues.IDLE_kP,0, Constants.ShooterValues.IDLE_kD);

    private SHOOTER_MODE mode = SHOOTER_MODE.IDLE, lastMode = SHOOTER_MODE.IDLE;

    public Shooter(){
        DashboardManager.addTab(this);
        DashboardManager.getTab(this).add(this);

        primaryMotor = new CANSparkMax(Constants.ShooterValues.PRIMARY_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        secondaryMotor = new CANSparkMax(Constants.ShooterValues.SECONDARY_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

        hoodMotor = new CANSparkMax(Constants.ShooterValues.HOOD_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        turretMotor = new CANSparkMax(Constants.ShooterValues.TURRET_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

        primaryMotor.wipeSettings();
        secondaryMotor.wipeSettings();

        secondaryMotor.follow(primaryMotor);

        hoodMotor.wipeSettings();
        turretMotor.wipeSettings();

        NeutralMode flywheelMode = NeutralMode.kCoast;
        primaryMotor.setNeutralMode(flywheelMode);
        secondaryMotor.setNeutralMode(flywheelMode);

        NeutralMode shooterMode = NeutralMode.kCoast;
        hoodMotor.setNeutralMode(shooterMode);
        turretMotor.setNeutralMode(shooterMode);

        primaryMotor.setInverted(false);
        secondaryMotor.setInverted(false);

        setCurrentLimit(40);

        dashboard_wheelRPM = DashboardManager.addTabItem(this, "RPM", 0.0);
        dashboard_PID = DashboardManager.addTabItem(this, "PID", 0.0);

        setDefaultCommand(new TemplateCommand(this));

        initSuccessful = true;
    }

    @Override
    public String getDashboardTabName () {
        return "Shooter";
    }

    @Override
    public void periodic() {
        updatePIDMode();
    }

    @Override
    public void updateDashboardData () {
        dashboard_wheelRPM.setDouble(primaryMotor.getSensorVelocity());
    }

    public void setShooterMode (SHOOTER_MODE mode) {
        setMode(mode);
    }

    private void setMode (SHOOTER_MODE mode) {
        lastMode = this.mode;
        this.mode = mode;
    }

    public SHOOTER_MODE getMode () {
        return mode;
    }

    private void updatePIDMode () {
        if (mode == SHOOTER_MODE.SPINNING_UP){
            pid.setPID(Constants.ShooterValues.SPIN_UP_kP, 0, Constants.ShooterValues.SPIN_UP_kD);
        } else if (mode == SHOOTER_MODE.SPINNING_DOWN) {
            pid.setPID(Constants.ShooterValues.SPIN_DOWN_kP, 0, Constants.ShooterValues.SPIN_DOWN_kD);
        } else if (mode == SHOOTER_MODE.SHOOTING){
            pid.setPID(Constants.ShooterValues.SHOOTING_kP, 0, Constants.ShooterValues.SHOOTING_kD);
        } else {
            pid.setPID(Constants.ShooterValues.IDLE_kP, 0, Constants.ShooterValues.SHOOTING_kD);
        }
    }

    public void setRPM(double rpm) {
        if (rpm > primaryMotor.getSensorVelocity() + 1000) {
            setShooterMode(SHOOTER_MODE.SPINNING_UP);
        } else if (rpm < primaryMotor.getSensorVelocity() - 1000) {
            setShooterMode(SHOOTER_MODE.SPINNING_DOWN);
        } else if (rpm > 1000){
            setShooterMode(SHOOTER_MODE.IDLE);
        }
        double power = pid.calculate(primaryMotor.getSensorVelocity(), rpm);
        dashboard_PID.setDouble(power);
        primaryMotor.set(power);
    }

    public enum SHOOTER_MODE {
        SPINNING_UP,
        SPINNING_DOWN,
        SHOOTING,
        IDLE
    }

    public void stopShooter() {
        primaryMotor.set(0);
    }

    public void setCurrentLimit(int limit) {
        primaryMotor.limitInputCurrent(limit);
        secondaryMotor.limitInputCurrent(limit);
    }

}
