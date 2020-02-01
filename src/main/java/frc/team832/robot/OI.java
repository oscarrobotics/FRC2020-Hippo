package frc.team832.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunEndCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.team832.lib.driverinput.controllers.*;
import frc.team832.lib.driverinput.oi.DriverOI;
import frc.team832.lib.driverinput.oi.SticksDriverOI;
import frc.team832.lib.driverinput.oi.XboxDriverOI;
import frc.team832.robot.commands.PrepareShooter;
import frc.team832.robot.commands.ShootCommandGroup;
import frc.team832.robot.subsystems.SuperStructure;

import static frc.team832.robot.Robot.*;

@SuppressWarnings("WeakerAccess")
public class OI {
	public final DriverOI driverOI;
	public static final boolean isSticks = RobotBase.isReal();
	public static final StratComInterface stratComInterface = new StratComInterface(isSticks ? 2 : 1);

	public Attack3 leftStick;
	public Extreme3DPro rightStick;

	public OI() {
		if (isSticks) {
			driverOI = new SticksDriverOI();
			leftStick = ((SticksDriverOI)driverOI).leftStick;
			rightStick = ((SticksDriverOI)driverOI).rightStick;
		} else {
			driverOI = new XboxDriverOI();
		}

		configureButtonBindings();
	}

	private void configureButtonBindings() {
		// do commands here
		stratComInterface.getSCPlus().whileHeld(new StartEndCommand(climber::windWinch, climber::stopWinch, Robot.climber));
		stratComInterface.getSCMinus().whileHeld(new StartEndCommand(climber::unwindWinch, climber::stopWinch, Robot.climber));

		stratComInterface.getArcadeWhiteLeft().whileHeld(new ConditionalCommand(
				new StartEndCommand(climber::unwindLeftWinch, climber::stopLeftWinch, climber),
				new StartEndCommand(climber::windLeftWinch, climber::stopLeftWinch, climber),
				stratComInterface.getSCSideTop()::get)
		);
		stratComInterface.getArcadeWhiteRight().whileHeld(new ConditionalCommand(
				new StartEndCommand(climber::unwindRightWinch, climber::stopRightWinch, climber),
				new StartEndCommand(climber::windRightWinch, climber::stopRightWinch, climber),
				stratComInterface.getSCSideTop()::get)
		);

		stratComInterface.getSCSideMid().whileHeld(new StartEndCommand(superStructure::intake, superStructure::idleIntake));
		stratComInterface.getSCSideBot().whileHeld(new StartEndCommand(superStructure::outtake, superStructure::idleIntake));

		stratComInterface.getArcadeBlackRight().whenPressed(new PrepareShooter(superStructure, pneumatics, shooter, spindexer));
		stratComInterface.getArcadeBlackLeft().whileHeld(new ShootCommandGroup(superStructure, pneumatics, shooter, spindexer));

		stratComInterface.getSC2().whileHeld(new RunEndCommand(pneumatics::extendWOFManipulator, pneumatics::retractWOFManipulator, Robot.pneumatics));
		stratComInterface.getSC1().whileHeld(new StartEndCommand(wheelOfFortune::spinCounterClockWise, wheelOfFortune::stopSpin, Robot.wheelOfFortune));
		stratComInterface.getSC3().whileHeld(new StartEndCommand(wheelOfFortune::spinClockWise, wheelOfFortune::stopSpin, Robot.wheelOfFortune));
		stratComInterface.getSC6().whenPressed(new InstantCommand(wheelOfFortune::spinThreeRot, Robot.wheelOfFortune));
	}
}
