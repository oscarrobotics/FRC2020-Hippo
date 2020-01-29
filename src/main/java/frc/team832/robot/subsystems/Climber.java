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
        DashboardManager.addTab(this);
        DashboardManager.getTab(this).add(this);

        leftWinch = new CANSparkMax(Constants.ClimberValues.LEFT_WINCH_CAN_ID, Motor.kNEO);
        rightWinch = new CANSparkMax(Constants.ClimberValues.RIGHT_WINCH_CAN_ID, Motor.kNEO);

        leftWinch.wipeSettings();

        leftWinch.setNeutralMode(NeutralMode.kBrake);

        leftWinch.setInverted(false);
        leftWinch.setSensorPhase(true);

        leftWinch.limitInputCurrent(40);

        initSuccessful = true;
    }

    public boolean isInitSuccessful() { return initSuccessful; }

    public void unwindWinch() {
        pneumatics.unlockClimb();
        leftWinch.set(-Constants.ClimberValues.WINCH_POWER);
    }

    public void windWinch() {
        pneumatics.unlockClimb();
        leftWinch.set(Constants.ClimberValues.WINCH_POWER);
    }

    public void stopWinch() {
        leftWinch.set(0);
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
