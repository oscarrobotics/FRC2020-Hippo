package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.impl.SmartMCAttachedPDPSlot;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;

public class Climber extends SubsystemBase implements DashboardUpdatable {
    public final boolean initSuccessful;
    private final CANSparkMax winchMotor, deployMotor;

    private SmartMCAttachedPDPSlot winchSlot, deploySlot;

    private ProfiledPIDController extendPID = new ProfiledPIDController(Constants.ClimberValues.ExtendkP, 0, 0, Constants.ClimberValues.ExtendConstraints);

    public Climber(GrouchPDP pdp) {
        DashboardManager.addTab(this, this);

        winchMotor = new CANSparkMax(Constants.ClimberValues.WINCH_CAN_ID, Motor.kNEO);
        deployMotor = new CANSparkMax(Constants.ClimberValues.DEPLOY_CAN_ID, Motor.kNEO550);

        winchSlot = pdp.addDevice(Constants.ClimberValues.WINCH_PDP_PORT, winchMotor);
        deploySlot = pdp.addDevice(Constants.ClimberValues.DEPLOY_PDP_PORT, deployMotor);

        winchMotor.wipeSettings();
        deployMotor.wipeSettings();

        winchMotor.setNeutralMode(NeutralMode.kBrake);
        deployMotor.setNeutralMode(NeutralMode.kBrake);

        winchMotor.setInverted(false);
        winchMotor.setSensorPhase(true);

        deployMotor.setInverted(false);
        deployMotor.setSensorPhase(true);

        winchMotor.limitInputCurrent(40);
        deployMotor.limitInputCurrent(30);

        initSuccessful = winchMotor.getCANConnection() && deployMotor.getCANConnection();
    }

    public void unwindWinch() {
        winchMotor.set(-0.25);
    }

    public void windWinch() {
        winchMotor.set(0.25);
    }

    public void extendHook() {
        deployMotor.set(extendPID.calculate(deployMotor.getSensorPosition(), Constants.ClimberValues.MinExtend));
    }

    public void retractHook() {
        deployMotor.set(extendPID.calculate(deployMotor.getSensorPosition(), Constants.ClimberValues.Retract));
    }

    public void stopExtend() {
        deployMotor.set(0);
    }

    public void adjustHook(double slider) {
        double targetPos = OscarMath.clipMap(slider, -1, 1, Constants.ClimberValues.MinExtend, Constants.ClimberValues.MaxExtend);
        deployMotor.set(extendPID.calculate(deployMotor.getSensorPosition(), targetPos));
    }

    public void climbDown() {
//        pneumatics.unlockClimb();
        unwindWinch();
    }

    public void climbUp() {
        retractHook();
//        pneumatics.unlockClimb();
        windWinch();
    }

    public void stopClimb() {
        winchMotor.set(0);
//        pneumatics.lockClimb();
    }

    @Override
    public String getDashboardTabName() {
        return "Climber";
    }

    @Override
    public void updateDashboardData() {

    }
}
