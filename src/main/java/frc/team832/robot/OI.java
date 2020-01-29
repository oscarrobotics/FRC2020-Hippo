package frc.team832.robot;

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
	public static final boolean isSticks = true;
	public static final StratComInterface stratComInterface = new StratComInterface(isSticks ? 2 : 1);

	public final Attack3 leftStick;
	public final Extreme3DPro rightStick;

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
		stratComInterface.getArcadeBlackLeft().whileHeld(new StartEndCommand(climber::windWinch, climber::stopWinch, Robot.climber));
		stratComInterface.getArcadeWhiteLeft().whileHeld(new StartEndCommand(climber::unwindWinch, climber::stopWinch, Robot.climber));

		stratComInterface.getSCSideTop().whileHeld(new StartEndCommand(superStructure::intake, superStructure::idleIntake));
		stratComInterface.getSCSideBot().whileHeld(new StartEndCommand(superStructure::outtake, superStructure::idleIntake));

		stratComInterface.getArcadeBlackRight().whenPressed(new PrepareShooter(superStructure));
		stratComInterface.getArcadeWhiteRight().whileHeld(new ShootCommandGroup(superStructure));

		stratComInterface.getSC2().whileHeld(new RunEndCommand(pneumatics::extendWOFManipulator, pneumatics::retractWOFManipulator, Robot.pneumatics));
		stratComInterface.getSC1().whileHeld(new StartEndCommand(wheelOfFortune::spinCounterClockWise, wheelOfFortune::stopSpin, Robot.wheelOfFortune));
		stratComInterface.getSC1().whileHeld(new StartEndCommand(wheelOfFortune::spinClockWise, wheelOfFortune::stopSpin, Robot.wheelOfFortune));
		stratComInterface.getSC6().whenPressed(new InstantCommand(wheelOfFortune::spinThreeRot, Robot.wheelOfFortune));
	}
}