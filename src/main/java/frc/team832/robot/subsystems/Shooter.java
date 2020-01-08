package frc.team832.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.robot.Constants;
import frc.team832.robot.commands.TemplateCommand;

public class Shooter extends SubsystemBase {

    private boolean initSuccessful = false;

    private CANSparkMax ShooterMotorPrimary, ShooterMotorFollower; //if needed add hood motor

    PIDController pid = new PIDController(Constants.SHOOTER_SPIN_UP_kP,0,Constants.SHOOTER_SPIN_UP_kD);

    private SHOOTER_MODE mode = SHOOTER_MODE.IDLE, lastMode = SHOOTER_MODE.IDLE;

    public Shooter(){
        ShooterMotorPrimary = new CANSparkMax(Constants.SHOOTER_ID_PRIMARY, CANSparkMaxLowLevel.MotorType.kBrushless);
        ShooterMotorFollower = new CANSparkMax(Constants.SHOOTER_ID_SECONDARY, CANSparkMaxLowLevel.MotorType.kBrushless);

        ShooterMotorPrimary.wipeSettings();
        ShooterMotorFollower.wipeSettings();

        ShooterMotorFollower.follow(ShooterMotorPrimary);

        setDefaultCommand(new TemplateCommand(this));

        initSuccessful = true;
    }

    @Override
    public void periodic() {

        if(lastMode != mode){
            if (mode == SHOOTER_MODE.SHOOTING){
                pid.setPID(Constants.SHOOTING_kP, 0, Constants.SHOOTING_kD);
            } else if (mode == SHOOTER_MODE.SPINNING_UP) {
                pid.setPID(Constants.SHOOTER_SPIN_UP_kP, 0, Constants.SHOOTER_SPIN_UP_kD);
            } else if (mode == SHOOTER_MODE.SPINNING_DOWN) {
                pid.setPID(Constants.SHOOTER_SPIN_DOWN_kP, 0, Constants.SHOOTER_SPIN_DOWN_kD);
            } else {
                pid.setPID(0,0,0);
            }
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
        if (mode == SHOOTER_MODE.SHOOTING){
            pid.setPID(Constants.SHOOTING_kP, 0, Constants.SHOOTING_kD);
        } else if (mode == SHOOTER_MODE.SPINNING_UP) {
            pid.setPID(Constants.SHOOTER_SPIN_UP_kP, 0, Constants.SHOOTER_SPIN_UP_kD);
        }
    }

    public enum SHOOTER_MODE {
        SPINNING_UP,
        SPINNING_DOWN,
        SHOOTING,
        IDLE
    }


}
