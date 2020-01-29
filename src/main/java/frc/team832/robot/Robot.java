/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team832.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.PDPPortNumber;
import frc.team832.robot.subsystems.*;

public class Robot extends TimedRobot {

    public static final GrouchPDP pdp = new GrouchPDP(0);

    // Subsystems
    public static final Drivetrain drivetrain = new Drivetrain(pdp);
    public static final Intake intake = new Intake();
    public static final Shooter shooter = new Shooter();
    public static final Spindexer spindexer = new Spindexer();
    public static final Vision vision = new Vision();
    public static final Climber climber = new Climber();
    public static final Pneumatics pneumatics = new Pneumatics();
    public static final WheelOfFortune wheelOfFortune = new WheelOfFortune();
    public static final SuperStructure superStructure = new SuperStructure(intake, shooter, spindexer, pneumatics);

    public static final OI oi = new OI();

    private static final Notifier drivetrainTelemetryNotifier = new Notifier(drivetrain::updateDashboardData);
    private static final Notifier shooterTelemetryNotifier = new Notifier(shooter::updateDashboardData);

    @Override
    public void robotInit() {
        if (!drivetrain.isInitSuccessful()) {
            System.out.println("Drivetrain - init FAILED");
        } else if (intake.isInitSuccessful()) {
            System.out.println("Intake - init FAILED");
//    } else if (vision.isInitSuccessful()) {
//      System.out.println("Vision - init FAILED");
        } else if (shooter.isInitSuccessful()) {
            System.out.println("Shooter - init FAILED");
        } else if (spindexer.isInitSuccessful()) {
            System.out.println("Spindexer - init FAILED");
        } else if (climber.isInitSuccessful()) {
            System.out.println("Climber - init FAILED");
        } else if (pneumatics.isInitSuccessful()) {
            System.out.println("Pneumatics - init FAILED");
        } else if (wheelOfFortune.isInitSuccessful()) {
            System.out.println("WheelOfFortune - init FAILED");
        }
    }

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }
}
