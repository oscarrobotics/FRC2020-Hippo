package frc.team832.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunEndCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team832.lib.driverinput.controllers.*;
import frc.team832.lib.driverinput.oi.DriverOI;
import frc.team832.lib.driverinput.oi.OperatorInterface;
import frc.team832.lib.driverinput.oi.SticksDriverOI;
import frc.team832.lib.driverinput.oi.XboxDriverOI;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.commands.climb.StartClimbGroup;
import frc.team832.robot.commands.climb.StopClimbGroup;
import frc.team832.robot.subsystems.Spindexer;
import frc.team832.robot.subsystems.SuperStructure;


import static frc.team832.robot.Robot.*;

@SuppressWarnings("WeakerAccess")
public class OI {
	public final DriverOI driverOI;
	public static final boolean isSticks = RobotBase.isReal();
	public static final StratComInterface stratComInterface = new StratComInterface(isSticks ? 2 : 1);
	private final SuperStructure superStructure;

	public Attack3 leftStick;
	public Extreme3DPro rightStick;

	public Xbox360Controller testXbox;

	public OI(SuperStructure superStructure) {
		if (isSticks) {
			driverOI = new SticksDriverOI();
			leftStick = ((SticksDriverOI)driverOI).leftStick;
			rightStick = ((SticksDriverOI)driverOI).rightStick;
		} else {
			driverOI = new XboxDriverOI();
		}

		this.superStructure = superStructure;

		if (OperatorInterface.getConnectedControllerCount() > 1 && DriverStation.getInstance().isJoystickConnected(0)) {
			configTestingCommands();
		}

		if (DriverStation.getInstance().isJoystickConnected(3)) {
			testXbox = new Xbox360Controller(3);
			configTestXboxCommands();
		}
	}

	private void configureOperatorLayout() {
		stratComInterface.getSC1().whenHeld(superStructure.idleCommand);
		stratComInterface.getSC2().whenHeld(superStructure.targetingCommand);
		stratComInterface.getSC3().whenHeld(superStructure.shootOnTarget);
		stratComInterface.getSC6().whenHeld(superStructure.closeShoot);

		stratComInterface.getSingleToggle().whenHeld(new RunEndCommand(() -> climber.adjustHook(stratComInterface.getLeftSlider()), climber::stopExtend));
		stratComInterface.getSingleToggle().whenReleased(new InstantCommand(climber::retractHook));

		stratComInterface.getSCMinus().whileHeld(new StartClimbGroup(climber, false));
		stratComInterface.getSCMinus().whenReleased(new InstantCommand(climber::lockClimb));

//		stratComInterface.getSCSideTop().whenHeld(new StartEndCommand(wheelOfFortune::extendWOFManipulator, wheelOfFortune::retractWOFManipulator));
//		stratComInterface.getSC1().whenHeld(new StartEndCommand(wheelOfFortune::spinCounterclockwise, wheelOfFortune::stopSpin, wheelOfFortune));//torque vector up
//		stratComInterface.getSC3().whenHeld(new StartEndCommand(wheelOfFortune::spinClockwise, wheelOfFortune::stopSpin, wheelOfFortune));//torque vector down
//		stratComInterface.getSC2().whenHeld(new RunEndCommand(wheelOfFortune::spinThreeTimes, wheelOfFortune::stopSpin, wheelOfFortune));
	}

	private void configTestingCommands() {
//		stratComInterface.getSC1().whenHeld(superStructure.idleCommand);
//		stratComInterface.getSC2().whenHeld(superStructure.targetingCommand);
//		stratComInterface.getSC3().whenHeld(superStructure.shootOnTarget);
//		stratComInterface.getSC6().whenHeld(superStructure.closeShoot);

		stratComInterface.getSCSideTop().whenPressed(superStructure.extendIntake);
		stratComInterface.getSCSideTop().whenReleased(superStructure.retractIntake);

		stratComInterface.getSCSideBot().whenPressed(superStructure.extendOuttake);
		stratComInterface.getSCSideBot().whenReleased(superStructure.retractIntake);

		stratComInterface.getSCSideMid()
				.whenPressed(new InstantCommand(() -> intake.outtake(0.3)))
				.whenReleased(intake::stop);

	}

	private void configTestXboxCommands() {
		testXbox.aButton.whileHeld( new StartEndCommand(
				() -> {
					intake.extendIntake();
					intake.intake(0);
				},
				() -> {
					intake.retractIntake();
					intake.stop();
				}, intake
		));
	}
}
