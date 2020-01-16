package frc.team832.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.robot.Constants;
import frc.team832.robot.commands.TemplateCommand;

public class Shooter extends SubsystemBase implements DashboardUpdatable {

    private boolean initSuccessful = false;

    private CANSparkMax primaryMotor, secondaryMotor; //if needed add hood motor
    private NetworkTableEntry dashboard_primaryWheelRPM, dashboard_secondaryWheelRPM, dashboard_primaryPID, dashboard_secondaryPID;

    PIDController primaryPid = new PIDController(Constants.ShooterValues.IDLE_kP,0, Constants.ShooterValues.IDLE_kD);
    PIDController secondaryPid = new PIDController(Constants.ShooterValues.IDLE_kP,0, Constants.ShooterValues.IDLE_kD);

    private SHOOTER_MODE primaryMode = SHOOTER_MODE.IDLE, primaryLast = SHOOTER_MODE.IDLE, secondaryMode = SHOOTER_MODE.IDLE, secondaryLast = SHOOTER_MODE.IDLE;

    public Shooter(){
        DashboardManager.addTab(this);
        DashboardManager.getTab(this).add(this);

        primaryMotor = new CANSparkMax(Constants.ShooterValues.SHOOTER_ID_PRIMARY, CANSparkMaxLowLevel.MotorType.kBrushless);
        secondaryMotor = new CANSparkMax(Constants.ShooterValues.SHOOTER_ID_SECONDARY, CANSparkMaxLowLevel.MotorType.kBrushless);

        primaryMotor.wipeSettings();
        secondaryMotor.wipeSettings();

//        secondaryMotor.follow(primaryMotor);

        dashboard_primaryWheelRPM = DashboardManager.addTabItem(this, "Top RPM", 0.0);
        dashboard_secondaryWheelRPM = DashboardManager.addTabItem(this, "Bottom RPM", 0.0);
        dashboard_primaryPID = DashboardManager.addTabItem(this, "Top PID", 0.0);
        dashboard_secondaryPID = DashboardManager.addTabItem(this, "Bottom PID", 0.0);

        setDefaultCommand(new TemplateCommand(this));

        initSuccessful = true;
    }

    @Override
    public String getDashboardTabName () {
        return "Shooter";
    }

    @Override
    public void periodic() {
        updateTopPIDMode();
        updateBottomPIDMode();

    }

    @Override
    public void updateDashboardData () {
        dashboard_primaryWheelRPM.setDouble(primaryMotor.getSensorVelocity());
        dashboard_secondaryWheelRPM.setDouble(secondaryMotor.getSensorVelocity());

    }

    public void setShooterMode (SHOOTER_MODE mode) {
        setPrimaryMode(mode);
        setSecondaryMode(mode);
    }

    private void setPrimaryMode(SHOOTER_MODE mode) {
        primaryLast = this.primaryMode;
        this.primaryMode = mode;
    }

    private void setSecondaryMode(SHOOTER_MODE mode) {
        secondaryLast = this.secondaryMode;
        this.secondaryMode = mode;
    }

    public SHOOTER_MODE getPrimaryMode() {
        return primaryMode;
    }
    public SHOOTER_MODE getSecondaryMode() {
        return secondaryMode;
    }

    private void updateTopPIDMode () {
        if (primaryMode == SHOOTER_MODE.SPINNING_UP){
            primaryPid.setPID(Constants.ShooterValues.SPIN_UP_kP, 0, Constants.ShooterValues.SPIN_UP_kD);
        } else if (primaryMode == SHOOTER_MODE.SPINNING_DOWN) {
            primaryPid.setPID(Constants.ShooterValues.SPIN_DOWN_kP, 0, Constants.ShooterValues.SPIN_DOWN_kD);
        } else if (primaryMode == SHOOTER_MODE.SHOOTING){
            primaryPid.setPID(Constants.ShooterValues.SHOOTING_kP, 0, Constants.ShooterValues.SHOOTING_kD);
        } else {
            primaryPid.setPID(Constants.ShooterValues.IDLE_kP, 0, Constants.ShooterValues.SHOOTING_kD);
        }
    }

    private void updateBottomPIDMode () {
        if (secondaryMode == SHOOTER_MODE.SPINNING_UP){
            secondaryPid.setPID(Constants.ShooterValues.SPIN_UP_kP, 0, Constants.ShooterValues.SPIN_UP_kD);
        } else if (secondaryMode == SHOOTER_MODE.SPINNING_DOWN) {
            secondaryPid.setPID(Constants.ShooterValues.SPIN_DOWN_kP, 0, Constants.ShooterValues.SPIN_DOWN_kD);
        } else if (secondaryMode == SHOOTER_MODE.SHOOTING){
            secondaryPid.setPID(Constants.ShooterValues.SHOOTING_kP, 0, Constants.ShooterValues.SHOOTING_kD);
        } else {
            secondaryPid.setPID(Constants.ShooterValues.IDLE_kP, 0, Constants.ShooterValues.SHOOTING_kD);
        }
    }

    public void setTopRPM(double rpm) {
        if (rpm > primaryMotor.getSensorVelocity() + 1000) {
            setShooterMode(SHOOTER_MODE.SPINNING_UP);
        } else if (rpm < primaryMotor.getSensorVelocity() - 1000) {
            setShooterMode(SHOOTER_MODE.SPINNING_DOWN);
        } else {
            setShooterMode(SHOOTER_MODE.IDLE);
        }
        double power = primaryPid.calculate(primaryMotor.getSensorVelocity(), rpm);
        dashboard_primaryPID.setDouble(power);
        primaryMotor.set(power);
    }

    public void setBottomRPM(double rpm) {
        if (rpm > primaryMotor.getSensorVelocity() + 1000) {
            setShooterMode(SHOOTER_MODE.SPINNING_UP);
        } else if (rpm < primaryMotor.getSensorVelocity() - 1000) {
            setShooterMode(SHOOTER_MODE.SPINNING_DOWN);
        } else {
            setShooterMode(SHOOTER_MODE.IDLE);
        }
        double power = secondaryPid.calculate(secondaryMotor.getSensorVelocity(), rpm);
        dashboard_secondaryPID.setDouble(power);
        secondaryMotor.set(power);
    }

    public enum SHOOTER_MODE {
        SPINNING_UP,
        SPINNING_DOWN,
        SHOOTING,
        IDLE
    }

    public boolean isInitSuccessful(){
        return initSuccessful;
    }

}
