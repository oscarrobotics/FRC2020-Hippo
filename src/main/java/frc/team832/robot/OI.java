package frc.team832.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunEndCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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

	public OI(SuperStructure superStructure) {
		if (isSticks) {
			driverOI = new SticksDriverOI();
			leftStick = ((SticksDriverOI)driverOI).leftStick;
			rightStick = ((SticksDriverOI)driverOI).rightStick;
		} else {
			driverOI = new XboxDriverOI();
		}

		this.superStructure = superStructure;

		if (OperatorInterface.getConnectedControllerCount() > 1) {
			configTestingCommands();
		}
	}

	private void configureBrandonLayout() {

//		stratComInterface.getArcadeBlackRight().whenHeld(new StartEndCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.IDLE),
//				() -> superStructure.setState(SuperStructure.SuperstructureState.IDLE)));

//		stratComInterface.getArcadeBlackLeft().whenHeld(new StartEndCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.INTAKE),
//				() -> superStructure.setState(SuperStructure.SuperstructureState.IDLE)));
//		stratComInterface.getArcadeBlackLeft().whenHeld(new RunEndCommand(() -> superStructure.configureSpindexerRPMSlider(stratComInterface.getRightSlider()),
//				superStructure::setSpindexerIntakeRpmDefault));

//		stratComInterface.getArcadeWhiteLeft().whenHeld(new StartEndCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.TARGETING),
//				() -> superStructure.setState(SuperStructure.SuperstructureState.IDLE)));

//		stratComInterface.getArcadeWhiteRight().whenHeld(new StartEndCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.SHOOTING),
//				() -> superStructure.setState(SuperStructure.SuperstructureState.IDLE)));


//		stratComInterface.getSingleToggle().whenHeld(new RunEndCommand(() -> climber.adjustHook(stratComInterface.getLeftSlider()), climber::stopExtend));
//		stratComInterface.getSingleToggle().whenReleased(new InstantCommand(climber::retractHook));
//
//		stratComInterface.getSCPlus().whileHeld(new StartClimbGroup(climber, true));
//		stratComInterface.getSCPlus().whenReleased(new InstantCommand(climber::lockClimb));
//
//		stratComInterface.getSCMinus().whileHeld(new StartClimbGroup(climber, false));
//		stratComInterface.getSCMinus().whenReleased(new InstantCommand(climber::lockClimb));

//		stratComInterface.getSCSideTop().whenHeld(new StartEndCommand(wheelOfFortune::extendWOFManipulator, wheelOfFortune::retractWOFManipulator));
//		stratComInterface.getSC1().whenHeld(new StartEndCommand(wheelOfFortune::spinCounterclockwise, wheelOfFortune::stopSpin, wheelOfFortune));//torque vector up
//		stratComInterface.getSC3().whenHeld(new StartEndCommand(wheelOfFortune::spinClockwise, wheelOfFortune::stopSpin, wheelOfFortune));//torque vector down
//		stratComInterface.getSC2().whenHeld(new RunEndCommand(wheelOfFortune::spinThreeTimes, wheelOfFortune::stopSpin, wheelOfFortune));
	}

	private void configTestingCommands() {
		stratComInterface.getArcadeBlackRight().whenHeld(superStructure.idleCommand);
		stratComInterface.getArcadeWhiteLeft().whenHeld(superStructure.targetingCommand);
		stratComInterface.getArcadeWhiteRight().whenHeld(superStructure.shootOnTarget);
		stratComInterface.getArcadeBlackLeft().whenHeld(superStructure.closeShoot);

//		stratComInterface.getArcadeBlackLeft().whileHeld(new RunEndCommand(() -> shooter.setFlywheelVoltage(3), () -> shooter.setFlywheelVoltage(0), shooter));

		var testSpin = new RunEndCommand(spindexer::vibrate, spindexer::idle).withName("TestSpindexerCommand");
		stratComInterface.getSingleToggle().whenHeld(testSpin);

		stratComInterface.getDoubleToggleUp().whenHeld(new RunEndCommand(() -> turret.setHeadingSlider(stratComInterface.getRightSlider()), turret::setForward));
	}
}
