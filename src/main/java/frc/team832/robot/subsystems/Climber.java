package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.robot.Constants;

import static frc.team832.robot.Robot.pneumatics;

public class Climber extends SubsystemBase implements DashboardUpdatable {
    private boolean initSuccessful;
    private final CANSparkMax leftWinch, rightWinch;

    public Climber() {
        DashboardManager.addTab(this, this);

        leftWinch = new CANSparkMax(Constants.ClimberValues.LEFT_WINCH_CAN_ID, Motor.kNEO, false);
        rightWinch = new CANSparkMax(Constants.ClimberValues.RIGHT_WINCH_CAN_ID, Motor.kNEO, false);

        leftWinch.wipeSettings();
        rightWinch.wipeSettings();

        leftWinch.setNeutralMode(NeutralMode.kBrake);
        rightWinch.setNeutralMode(NeutralMode.kBrake);

        leftWinch.setInverted(false);
        leftWinch.setSensorPhase(true);

        rightWinch.setInverted(false);
        rightWinch.setSensorPhase(true);

        setCurrentLimit(40);

        initSuccessful = true;
    }

    private void setCurrentLimit (int limit) {
        leftWinch.limitInputCurrent(limit);
        rightWinch.limitInputCurrent(limit);
    }

    public boolean isInitSuccessful() { return initSuccessful; }

    public void unwindLeftWinch() {
        leftWinch.set(-Constants.ClimberValues.WINCH_POWER);
    }

    public void unwindRightWinch() {
        rightWinch.set(-Constants.ClimberValues.WINCH_POWER);
    }

    public void unwindWinch() {
        pneumatics.unlockClimb();
        unwindLeftWinch();
        unwindRightWinch();
    }

    public void windLeftWinch() {
        leftWinch.set(Constants.ClimberValues.WINCH_POWER);
    }

    public void windRightWinch() {
        rightWinch.set(Constants.ClimberValues.WINCH_POWER);
    }

    public void windWinch() {
        pneumatics.unlockClimb();
        windLeftWinch();
        windRightWinch();
    }

    public void stopLeftWinch() {
        leftWinch.set(0);
    }

    public void stopRightWinch() {
        rightWinch.set(0);
    }

    public void stopWinch() {
        pneumatics.lockClimb();
        stopLeftWinch();
        stopRightWinch();
    }

    @Override
    public String getDashboardTabName() {
        return "Climber";
    }

    @Override
    public void updateDashboardData() {

    }
}
