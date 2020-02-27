package frc.team832.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team832.lib.CANDevice;
import frc.team832.lib.OscarTimedRobot;
import frc.team832.lib.control.PCM;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.robot.subsystems.*;
import frc.team832.robot.utilities.state.SpindexerStatus;

@SuppressWarnings("WeakerAccess")
public class Robot extends OscarTimedRobot {

    public static final GrouchPDP pdp = new GrouchPDP(0);
    private static final Compressor pcm = new Compressor(0);

    // Subsystems
    static final Drivetrain drivetrain = new Drivetrain(pdp);
    static final Vision vision = new Vision(drivetrain);
    static final Intake intake = new Intake(pdp);
    static final Shooter shooter = new Shooter(pdp, vision);
    static final Spindexer spindexer = new Spindexer(pdp);
    static final Climber climber = new Climber(pdp);
    static final WheelOfFortune wheelOfFortune = new WheelOfFortune();
    static final SuperStructure superStructure = new SuperStructure(intake, shooter, spindexer);


    public static final OI oi = new OI();

    private static final Notifier drivetrainTelemetryNotifier = new Notifier(drivetrain::updateDashboardData);
    private static final Notifier shooterTelemetryNotifier = new Notifier(shooter::updateDashboardData);
    private static final Notifier intakeTelemetryNotifier = new Notifier(intake::updateDashboardData);
    private static final Notifier visionTelemetryNotifier = new Notifier(vision::updateDashboardData);

    @Override
    public void robotInit() {

        pcm.setClosedLoopControl(true);

        if (drivetrain.initSuccessful) {
            System.out.println("Drivetrain - init OK");
            drivetrainTelemetryNotifier.startPeriodic(0.02);
        } else {
            System.out.println("Drivetrain - init FAILED");
        }

        if (intake.initSuccessful) {
            System.out.println("Intake - init OK");
            intakeTelemetryNotifier.startPeriodic(0.02);
        } else {
            System.out.println("Intake - init FAILED");
        }

        if (vision.initSuccessful) {
            System.out.println("Vision - init OK");
            visionTelemetryNotifier.startPeriodic(0.02);
        } else {
            System.out.println("Vision - init FAILED");
        }

        if (shooter.initSuccessful) {
            System.out.println("Shooter - init OK");
            shooterTelemetryNotifier.startPeriodic(0.02);
        } else {
            System.out.println("Shooter - init FAILED");
            shooterTelemetryNotifier.startPeriodic(0.02);

        }

        if (spindexer.initSuccessful) {
            System.out.println("Spindexer - init OK");
        } else {
            System.out.println("Spindexer - init FAILED");
        }

        if (climber.initSuccessful) {
            System.out.println("Climber - init OK");
        } else {
            System.out.println("Climber - init FAILED");
        }

        if (wheelOfFortune.initSuccessful) {
            System.out.println("WheelOfFortune - init OK");
        } else {
            System.out.println("WheelOfFortune - init FAILED");
        }

        CANDevice.printMissingDevices();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void disabledInit() {
        NeutralMode mode = NeutralMode.kCoast;
        drivetrain.setNeutralMode(mode);
        shooter.setFlyheelNeutralMode(mode);
        shooter.setTurretNeutralMode(mode);
        spindexer.setNeutralMode(mode);
        climber.lockClimb();
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        NeutralMode mode = NeutralMode.kBrake;
        drivetrain.setNeutralMode(mode);
        shooter.setFlyheelNeutralMode(mode);
        shooter.setTurretNeutralMode(mode);
        shooter.holdTurretPosition();
        spindexer.setNeutralMode(mode);
        climber.zeroDeploy();
    }

    @Override
    public void teleopPeriodic() {
    }
}
