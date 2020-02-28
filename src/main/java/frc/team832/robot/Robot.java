package frc.team832.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team832.lib.CANDevice;
import frc.team832.lib.OscarTimedRobot;
import frc.team832.lib.control.PCM;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.robot.commands.ShootCommandGroup;
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
    static final Shooter shooter = new Shooter(pdp);
    static final Spindexer spindexer = new Spindexer(pdp);
    static final Turret turret = new Turret(pdp);
    static final Climber climber = new Climber(pdp);
    static final WheelOfFortune wheelOfFortune = new WheelOfFortune();
    static final SuperStructure superStructure = new SuperStructure(intake, shooter, spindexer, turret, vision);

    public static final OI oi = new OI(superStructure);

    private static final Notifier drivetrainTelemetryNotifier = new Notifier(drivetrain::updateDashboardData);
    private static final Notifier shooterTelemetryNotifier = new Notifier(shooter::updateDashboardData);
    private static final Notifier intakeTelemetryNotifier = new Notifier(intake::updateDashboardData);
    private static final Notifier turretTelemetryNotifier = new Notifier(turret::updateDashboardData);
    private static final Notifier visionTelemetryNotifier = new Notifier(vision::updateDashboardData);
    private static final Notifier superStructureTelemetryNotifier = new Notifier(superStructure::updateDashboardData);

    private Command autoCommand;

    @Override
    public void robotInit() {

        pcm.setClosedLoopControl(true);

        if (drivetrain.initSuccessful) {
            System.out.println("Drivetrain - init OK");
        } else {
            System.out.println("Drivetrain - init FAILED");
        }
        drivetrainTelemetryNotifier.startPeriodic(0.02);

        if (intake.initSuccessful) {
            System.out.println("Intake - init OK");
        } else {
            System.out.println("Intake - init FAILED");
        }
        intakeTelemetryNotifier.startPeriodic(0.02);

        if (vision.initSuccessful) {
            System.out.println("Vision - init OK");
        } else {
            System.out.println("Vision - init FAILED");
        }
        visionTelemetryNotifier.startPeriodic(0.02);

        if (shooter.initSuccessful) {
            System.out.println("Shooter - init OK");
        } else {
            System.out.println("Shooter - init FAILED");
        }
        shooterTelemetryNotifier.startPeriodic(0.02);

        if (turret.initSuccessful) {
            System.out.println("Turret - init OK");
        } else {
            System.out.println("Turret - init FAILED");
        }
        turretTelemetryNotifier.startPeriodic(0.02);

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

        superStructureTelemetryNotifier.startPeriodic(0.02);

        CANDevice.printMissingDevices();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        NeutralMode mode = NeutralMode.kBrake;
        drivetrain.setNeutralMode(mode);
        shooter.setFlyheelNeutralMode(mode);
        shooter.setFeederNeutralMode(mode);
        turret.holdTurretPosition();
        spindexer.setNeutralMode(mode);
        turret.setNeutralMode(mode);
        shooter.setHood(2.7);
        climber.zeroDeploy();
        autoCommand = new ShootCommandGroup(superStructure);
        autoCommand.schedule();
    }

    @Override
    public void disabledInit() {
        NeutralMode mode = NeutralMode.kCoast;
        drivetrain.setNeutralMode(mode);
        shooter.setFlyheelNeutralMode(mode);
        shooter.setFeederNeutralMode(mode);
        spindexer.setNeutralMode(mode);
        turret.setNeutralMode(mode);
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
        autoCommand.cancel();
        NeutralMode mode = NeutralMode.kBrake;
        drivetrain.setNeutralMode(mode);
        shooter.setFlyheelNeutralMode(mode);
        shooter.setFeederNeutralMode(mode);
        turret.holdTurretPosition();
        spindexer.setNeutralMode(mode);
        turret.setNeutralMode(mode);
        shooter.setHood(2.7);
        climber.zeroDeploy();
    }

    @Override
    public void teleopPeriodic() {
    }
}
