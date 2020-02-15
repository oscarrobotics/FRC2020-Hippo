/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team832.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.AddressableLED;
import edu.wpi.first.wpilibj2.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.robot.subsystems.*;

public class Robot extends TimedRobot {

    public static final GrouchPDP pdp = new GrouchPDP(0);

    // Subsystems
    public static final Drivetrain drivetrain = new Drivetrain(pdp);
    public static final Vision vision = new Vision(drivetrain);
    public static final Intake intake = new Intake(pdp);
    public static final Shooter shooter = new Shooter(pdp, vision);
    public static final Spindexer spindexer = new Spindexer(pdp);
    public static final Climber climber = new Climber(pdp);
    public static final Pneumatics pneumatics = new Pneumatics();
    public static final WheelOfFortune wheelOfFortune = new WheelOfFortune();
    public static final SuperStructure superStructure = new SuperStructure(intake, shooter, spindexer, pneumatics);

    public static final OI oi = new OI();

    private static final Notifier drivetrainTelemetryNotifier = new Notifier(drivetrain::updateDashboardData);
    private static final Notifier shooterTelemetryNotifier = new Notifier(shooter::updateDashboardData);

    AddressableLED led = new AddressableLED(Constants.LEDValues.LED_PWM_PORT);
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(16);

    @Override
    public void robotInit() {
        if (!drivetrain.isInitSuccessful()) {
            System.out.println("Drivetrain - init FAILED");
            drivetrainTelemetryNotifier.startPeriodic(0.02);
        }

        if (intake.isInitSuccessful()) {
            System.out.println("Intake - init FAILED");
        }

        if (vision.isInitSuccessful()) {
            System.out.println("Vision - init FAILED");
        }

        if (shooter.isInitSuccessful()) {
            System.out.println("Shooter - init FAILED");
            shooterTelemetryNotifier.startPeriodic(0.02);
        }

        if (spindexer.isInitSuccessful()) {
            System.out.println("Spindexer - init FAILED");
        }

        if (climber.isInitSuccessful()) {
            System.out.println("Climber - init FAILED");
        }

        if (pneumatics.isInitSuccessful()) {
            System.out.println("Pneumatics - init FAILED");
        }

        if (wheelOfFortune.isInitSuccessful()) {
            System.out.println("WheelOfFortune - init FAILED");
        }

        led.setLength(ledBuffer.getLength());
        for (int i = 0; i < ledBuffer.getLength(); i++)
            ledBuffer.setRGB(i, 0, 255, 0);

        led.setData(ledBuffer);
        led.start();
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
        shooter.setTurretMode(mode);
        pneumatics.lockClimb();
    }

    @Override
    public void disabledPeriodic() {

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
