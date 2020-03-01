package frc.team832.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunEndCommand;
import frc.team832.lib.driverinput.controllers.*;
import frc.team832.lib.driverinput.oi.DriverOI;
import frc.team832.lib.driverinput.oi.SticksDriverOI;
import frc.team832.lib.driverinput.oi.XboxDriverOI;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.commands.ClimbGroup;
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

		configureBrandonLayout();
	}

	private void configureBrandonLayout() {

		stratComInterface.getArcadeBlackRight().whenHeld(new RunEndCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.IDLE),
				() -> superStructure.setState(SuperStructure.SuperstructureState.IDLE)));

		stratComInterface.getArcadeBlackLeft().whenHeld(new RunEndCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.INTAKE),
				() -> superStructure.setState(SuperStructure.SuperstructureState.IDLE)));
		stratComInterface.getArcadeBlackLeft().whenHeld(new RunEndCommand(() -> superStructure.configureIntakeRPMSlider(stratComInterface.getRightSlider(),
				stratComInterface.getLeftSlider(), stratComInterface.getKeySwitch().get()), superStructure::setSpindexerIntakeRpmDefault));

		stratComInterface.getArcadeWhiteLeft().whenHeld(new RunEndCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.TARGETING),
				() -> superStructure.setState(SuperStructure.SuperstructureState.IDLE)));
		stratComInterface.getArcadeWhiteLeft().whenHeld(new RunEndCommand(() -> superStructure.setShooterParametersSlider(stratComInterface.getLeftSlider(), stratComInterface.getRightSlider(), stratComInterface.getKeySwitch().get()),
				superStructure::setShooterParametersDefault));

		stratComInterface.getArcadeWhiteRight().whenHeld(new RunEndCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.SHOOTING),
				() -> superStructure.setState(SuperStructure.SuperstructureState.IDLE)));
		stratComInterface.getArcadeWhiteRight().whenHeld(new RunEndCommand(() -> superStructure.setShooterParametersSlider(stratComInterface.getLeftSlider(), stratComInterface.getRightSlider(), stratComInterface.getKeySwitch().get()),
				superStructure::setShooterParametersDefault));


		stratComInterface.getSingleToggle().whenHeld(new RunEndCommand(() -> climber.adjustHook(stratComInterface.getLeftSlider()), climber::stopExtend));
		stratComInterface.getSingleToggle().whenReleased(new InstantCommand(climber::retractHook));

		stratComInterface.getSCPlus().whileHeld(new ClimbGroup(climber, true));
		stratComInterface.getSCPlus().whenReleased(new InstantCommand(climber::lockClimb));

		stratComInterface.getSCMinus().whileHeld(new ClimbGroup(climber, false));
		stratComInterface.getSCMinus().whenReleased(new InstantCommand(climber::lockClimb));

//		stratComInterface.getSCSideTop().whenHeld(new StartEndCommand(wheelOfFortune::extendWOFManipulator, wheelOfFortune::retractWOFManipulator));
//		stratComInterface.getSC1().whenHeld(new StartEndCommand(wheelOfFortune::spinCounterclockwise, wheelOfFortune::stopSpin, wheelOfFortune));//torque vector up
//		stratComInterface.getSC3().whenHeld(new StartEndCommand(wheelOfFortune::spinClockwise, wheelOfFortune::stopSpin, wheelOfFortune));//torque vector down
//		stratComInterface.getSC2().whenHeld(new RunEndCommand(wheelOfFortune::spinThreeTimes, wheelOfFortune::stopSpin, wheelOfFortune));
	}

	private void configTestingCommands() {
//		stratComInterface.getArcadeBlackRight().whenHeld(new RunEndCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.IDLE),
//				() -> superStructure.setState(SuperStructure.SuperstructureState.IDLE)));
//
//		stratComInterface.getArcadeBlackLeft().whenHeld(new RunEndCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.INTAKE),
//				() -> superStructure.setState(SuperStructure.SuperstructureState.IDLE)));
//		stratComInterface.getArcadeBlackLeft().whenHeld(new RunEndCommand(() -> superStructure.setSpindexerIntakeRPMSlider(stratComInterface.getRightSlider()),
//				superStructure::setSpindexerIntakeRpmDefault));
//
//		stratComInterface.getArcadeWhiteLeft().whenHeld(new RunEndCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.TARGETING),
//				() -> superStructure.setState(SuperStructure.SuperstructureState.IDLE)));
//		stratComInterface.getArcadeWhiteLeft().whenHeld(new RunEndCommand(() -> superStructure.setShooterParametersSlider(stratComInterface.getLeftSlider(), stratComInterface.getRightSlider(), stratComInterface.getKeySwitch().get()),
//				superStructure::setShooterParametersDefault));
//
//		stratComInterface.getArcadeWhiteRight().whenHeld(new RunEndCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.SHOOTING),
//				() -> superStructure.setState(SuperStructure.SuperstructureState.IDLE)));
//		stratComInterface.getArcadeWhiteRight().whenHeld(new RunEndCommand(() -> superStructure.setShooterParametersSlider(stratComInterface.getLeftSlider(), stratComInterface.getRightSlider(), stratComInterface.getKeySwitch().get()),
//				superStructure::setShooterParametersDefault));

		stratComInterface.getDoubleToggleUp().whenHeld(new RunEndCommand(() -> shooter.setFeedRPM(OscarMath.map(
				stratComInterface.getRightSlider(), -1, 1, 0, 4000)), () -> shooter.setFeedRPM(0)));
	}
}

