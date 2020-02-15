package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.impl.SmartMCAttachedPDPSlot;
import frc.team832.robot.Constants;

import static frc.team832.robot.Robot.pneumatics;

public class Climber extends SubsystemBase implements DashboardUpdatable {
    private boolean initSuccessful;
    private final CANSparkMax winch, deploy;

    private SmartMCAttachedPDPSlot winchSlot, deploySlot;

    private PIDController extendPID = new PIDController(Constants.ClimberValues.ExtendkP, 0, 0);

    public Climber(GrouchPDP pdp) {
        DashboardManager.addTab(this, this);

        winch = new CANSparkMax(Constants.ClimberValues.WINCH_CAN_ID, Motor.kNEO);
        deploy = new CANSparkMax(Constants.ClimberValues.DEPLOY_CAN_ID, Motor.kNEO550);

        winchSlot = pdp.addDevice(Constants.ClimberValues.WINCH_PDP_PORT, winch);
        deploySlot = pdp.addDevice(Constants.ClimberValues.DEPLOY_PDP_PORT, deploy);

        winch.wipeSettings();
        deploy.wipeSettings();

        winch.setNeutralMode(NeutralMode.kBrake);
        deploy.setNeutralMode(NeutralMode.kBrake);

        winch.setInverted(false);
        winch.setSensorPhase(true);

        deploy.setInverted(false);
        deploy.setSensorPhase(true);

        winch.limitInputCurrent(40);
        deploy.limitInputCurrent(30);

        initSuccessful = true;
    }

    public boolean isInitSuccessful() { return initSuccessful; }

    public void unwindWinch() {
        winch.set(-0.25);
    }

    public void windWinch() {
        winch.set(0.25);
    }

    public void extendHook() {
        double power = extendPID.calculate(deploy.getSensorPosition(), Constants.ClimberValues.MaxExtension);
//        deploy.set();
    }

    public void climbDown() {
        pneumatics.unlockClimb();
        unwindWinch();
    }

    public void climbUp() {
        pneumatics.unlockClimb();
        windWinch();
    }

    public void stopWinch() {
        winch.set(0);
    }

    public void stopClimb() {
        pneumatics.lockClimb();
        stopWinch();
    }

    @Override
    public String getDashboardTabName() {
        return "Climber";
    }

    @Override
    public void updateDashboardData() {

    }
}
