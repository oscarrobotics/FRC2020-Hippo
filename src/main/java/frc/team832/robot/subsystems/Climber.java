package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardWidget;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.impl.SmartMCAttachedPDPSlot;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;

public class Climber extends SubsystemBase {
    public final boolean initSuccessful;
    private final CANSparkMax winchMotor, deployMotor;

    private Solenoid climbLock;

    private double extendTarget = Constants.ClimberValues.Retract;
    private double climbPower = 0;

    private SmartMCAttachedPDPSlot winchSlot, deploySlot;

    private final NetworkTableEntry dashboard_isSafe, dashboard_deployTarget, dashboard_deployPosition, dashboard_deployPIDEffort, dashboard_atDeployTarget, dashboard_deployError;

    private ProfiledPIDController extendPID = new ProfiledPIDController(Constants.ClimberValues.ExtendkP, 0, 0, Constants.ClimberValues.ExtendConstraints);

    private SlewRateLimiter climbRamp = new SlewRateLimiter(1);

    public Climber(GrouchPDP pdp) {
        setName("Climber");
        DashboardManager.addTab(this);

        climbLock = new Solenoid(Constants.PneumaticsValues.PCM_MODULE_NUM, Constants.PneumaticsValues.CLIMB_LOCK_SOLENOID_ID);

        winchMotor = new CANSparkMax(Constants.ClimberValues.WINCH_CAN_ID, Motor.kNEO);
        deployMotor = new CANSparkMax(Constants.ClimberValues.DEPLOY_CAN_ID, Motor.kNEO550);

        winchSlot = pdp.addDevice(Constants.ClimberValues.WINCH_PDP_PORT, winchMotor);
        deploySlot = pdp.addDevice(Constants.ClimberValues.DEPLOY_PDP_PORT, deployMotor);

        winchMotor.wipeSettings();
        deployMotor.wipeSettings();

        deployMotor.rezeroSensor();

        extendPID.setTolerance(0.5);

        winchMotor.setNeutralMode(NeutralMode.kBrake);
        setDeployNeutralMode(NeutralMode.kBrake);

        winchMotor.setInverted(false);
        winchMotor.setSensorPhase(true);

        deployMotor.setInverted(false);
        deployMotor.setSensorPhase(true);

        winchMotor.limitInputCurrent(40);
        deployMotor.limitInputCurrent(30);

        dashboard_isSafe = DashboardManager.addTabItem(this, "Is Safe", false, DashboardWidget.BooleanBox);
        dashboard_deployTarget = DashboardManager.addTabItem(this, "Deploy Target Pos", 0);
        dashboard_deployPosition = DashboardManager.addTabItem(this, "Deploy Actual Pos", 0);
        dashboard_deployPIDEffort = DashboardManager.addTabItem(this, "Deploy PID Effort", 0.0);
        dashboard_atDeployTarget = DashboardManager.addTabItem(this, "At Extend Target", false, DashboardWidget.BooleanBox);
        dashboard_deployError = DashboardManager.addTabItem(this, "Deploy Error", 0);


        initSuccessful = winchMotor.getCANConnection() && deployMotor.getCANConnection();
    }

    @Override
    public void periodic() {
        runExtendPID();
        runClimbPID();
        dashboard_isSafe.setBoolean(isWinchSafe());
        dashboard_deployTarget.setDouble(extendTarget);
        dashboard_deployPosition.setDouble(deployMotor.getSensorPosition());
        dashboard_atDeployTarget.setBoolean(extendPID.atSetpoint());
        dashboard_deployError.setDouble(extendPID.getPositionError());
    }

    public void unwindWinch() { climbPower = -0.2; }

    public void windWinch() { climbPower = 0.75; }

    public boolean isWinchSafe() {
        return deployMotor.getSensorPosition() > Constants.ClimberValues.MinExtend;
    }

    public void retractHook() { extendTarget = Constants.ClimberValues.Retract; }

    public void stopExtend() {
        deployMotor.set(0);
    }

    public void adjustHook(double slider) {
        extendTarget = OscarMath.clipMap(slider, -1, 1, 0, Constants.ClimberValues.MaxExtend);
    }

    private void runExtendPID() {
        double pow = extendPID.calculate(deployMotor.getSensorPosition(), extendTarget);
        if (extendPID.atSetpoint()) {
            pow = 0;
        }
        deployMotor.set(pow);
        dashboard_deployPIDEffort.setDouble(pow);
    }

    private void runClimbPID() { winchMotor.set(climbRamp.calculate(climbPower)); }

    public void stopClimb() {
        climbPower = 0;
    }

    public void lockClimb() {
        climbLock.set(false);
    }

    public void unlockClimb() { climbLock.set(true); }

    public void zeroDeploy() {
        deployMotor.rezeroSensor();
        extendTarget = 0;
    }

    public void setDeployNeutralMode(NeutralMode mode) {
        deployMotor.setNeutralMode(mode);
    }
}
