package frc.team832.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team832.lib.CANDevice;
import frc.team832.lib.OscarTimedRobot;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.robot.commands.auto.BasicAutonomous;
import frc.team832.robot.commands.auto.DumbPathAuto;
import frc.team832.robot.subsystems.*;

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
//    static final WheelOfFortune wheelOfFortune = new WheelOfFortune();
    static final SuperStructure superStructure = new SuperStructure(intake, shooter, spindexer, turret, vision);

    public static final OI oi = new OI(superStructure);

    private Command autoCommand;

    @Override
    public void robotInit() {
        pcm.setClosedLoopControl(true);

        if (drivetrain.initSuccessful) {
            System.out.println("Drivetrain - init OK");
        } else {
            System.out.println("Drivetrain - init FAILED");
        }

        if (intake.initSuccessful) {
            System.out.println("Intake - init OK");
        } else {
            System.out.println("Intake - init FAILED");
        }

        if (vision.initSuccessful) {
            System.out.println("Vision - init OK");
        } else {
            System.out.println("Vision - init FAILED");
        }

        if (shooter.initSuccessful) {
            System.out.println("Shooter - init OK");
        } else {
            System.out.println("Shooter - init FAILED");
        }

        if (turret.initSuccessful) {
            System.out.println("Turret - init OK");
        } else {
            System.out.println("Turret - init FAILED");
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

//        if (wheelOfFortune.initSuccessful) {
//            System.out.println("WheelOfFortune - init OK");
//        } else {
//            System.out.println("WheelOfFortune - init FAILED");
//        }

        CANDevice.printMissingDevices();
//        autoCommand = new BasicAutonomous(superStructure, drivetrain);
        autoCommand = new DumbPathAuto(drivetrain);
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
        shooter.setFeederNeutralMode(mode);
        shooter.setFlyheelNeutralMode(NeutralMode.kCoast);
//        turret.holdTurretPosition();
        spindexer.setNeutralMode(mode);
        turret.setNeutralMode(mode);
        shooter.setHood(2.7);
        climber.zeroDeploy();
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
        drivetrain.driveMusic.play();
    }
}
