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
    private CANSparkMax winch;

    public Climber() {
        DashboardManager.addTab(this);
        DashboardManager.getTab(this).add(this);

        winch = new CANSparkMax(Constants.ClimberValues.WINCH_CAN_ID, Motor.kNEO);

        winch.wipeSettings();

        winch.setNeutralMode(NeutralMode.kBrake);

        winch.setInverted(false);
        winch.setSensorPhase(true);

        winch.limitInputCurrent(40);

        initSuccessful = true;
    }

    public boolean isInitSuccessful() { return initSuccessful; }

    public void unwindWinch() {
        pneumatics.unlockClimb();
        winch.set(-Constants.ClimberValues.WINCH_POWER);
    }

    public void windWinch() {
        pneumatics.unlockClimb();
        winch.set(Constants.ClimberValues.WINCH_POWER);
    }

    public void stopWinch() {
        winch.set(0);
        pneumatics.lockClimb();
    }

    @Override
    public String getDashboardTabName() {
        return "Climber";
    }

    @Override
    public void updateDashboardData() {

    }
}
