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

import static frc.team832.robot.Robot.oi;

public class Shooter extends SubsystemBase implements DashboardUpdatable {

    private boolean initSuccessful = false;

    private CANSparkMax primaryMotor, followerMotor; //if needed add hood motor
    private NetworkTableEntry dashboard_wheelRPM;

    PIDController pid = new PIDController(Constants.ShooterValues.SPIN_UP_kP,0, Constants.ShooterValues.SPIN_UP_kD);

    private SHOOTER_MODE mode = SHOOTER_MODE.IDLE, lastMode = SHOOTER_MODE.IDLE;

    public Shooter(){
        DashboardManager.addTab(this);
        DashboardManager.getTab(this).add(this);

        primaryMotor = new CANSparkMax(Constants.ShooterValues.SHOOTER_ID_PRIMARY, CANSparkMaxLowLevel.MotorType.kBrushless);
        followerMotor = new CANSparkMax(Constants.ShooterValues.SHOOTER_ID_SECONDARY, CANSparkMaxLowLevel.MotorType.kBrushless);

        primaryMotor.wipeSettings();
        followerMotor.wipeSettings();

        followerMotor.follow(primaryMotor);

        dashboard_wheelRPM = DashboardManager.addTabItem(this, "RPM", 0.0);

        setDefaultCommand(new TemplateCommand(this));

        initSuccessful = true;
    }

    @Override
    public String getDashboardTabName () {
        return "Shooter";
    }

    @Override
    public void periodic() {

        if(lastMode != mode){
            if (mode == SHOOTER_MODE.SHOOTING){
                pid.setPID(Constants.ShooterValues.SHOOTING_kP, 0, Constants.ShooterValues.SHOOTING_kD);
            } else if (mode == SHOOTER_MODE.SPINNING_UP) {
                pid.setPID(Constants.ShooterValues.SPIN_UP_kP, 0, Constants.ShooterValues.SPIN_UP_kD);
            } else if (mode == SHOOTER_MODE.SPINNING_DOWN) {
                pid.setPID(Constants.ShooterValues.SPIN_DOWN_kP, 0, Constants.ShooterValues.SPIN_DOWN_kD);
            } else {
                pid.setPID(0,0,0);
            }
        }
    }

    @Override
    public void updateDashboardData () {
        dashboard_wheelRPM.setDouble(primaryMotor.getSensorVelocity());
        if (oi.stratComInterface.getArcadeBlackLeft().get()) {
            setRPM(dashboard_wheelRPM.getDouble(0));
        }

    }

    public void setShooterMode (SHOOTER_MODE mode) {
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
        } else {
            setShooterMode(SHOOTER_MODE.IDLE);
        }
        primaryMotor.setVelocity(rpm);
    }

    public enum SHOOTER_MODE {
        SPINNING_UP,
        SPINNING_DOWN,
        SHOOTING,
        IDLE
    }


}
