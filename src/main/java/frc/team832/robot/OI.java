package frc.team832.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunEndCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.team832.lib.driverinput.controllers.*;
import frc.team832.lib.driverinput.oi.DriverOI;
import frc.team832.lib.driverinput.oi.SticksDriverOI;
import frc.team832.lib.driverinput.oi.XboxDriverOI;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.commands.ClimbGroup;
import frc.team832.robot.commands.PrepareShooter;
import frc.team832.robot.commands.ShootCommandGroup;
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

		configTestingCommands();
	}

	private void configureBrandonLayout() {
		stratComInterface.getSingleToggle().whenPressed(new InstantCommand(climber::extendHook));
		stratComInterface.getSingleToggle().whenHeld(new RunEndCommand(() -> climber.adjustHook(stratComInterface.getLeftSlider()), climber::stopExtend));
		stratComInterface.getSingleToggle().whenReleased(new InstantCommand(climber::retractHook));

		stratComInterface.getSCPlus().whenHeld(new StartEndCommand(climber::climbUp, climber::stopClimb, climber));
		stratComInterface.getSCMinus().whenHeld(new StartEndCommand(climber::climbDown, climber::stopClimb, climber));


		stratComInterface.getArcadeBlackLeft().whenHeld(new StartEndCommand(superStructure::intake, superStructure::idleIntake, superStructure, shooter, spindexer));
		stratComInterface.getArcadeBlackLeft().whenHeld(new StartEndCommand(superStructure::outtake, superStructure::idleIntake, superStructure, shooter, spindexer));

		stratComInterface.getArcadeWhiteLeft().whenPressed(new PrepareShooter(superStructure, shooter, spindexer, turret));
		stratComInterface.getArcadeWhiteRight().whenPressed(new ShootCommandGroup(superStructure));

		stratComInterface.getSCSideTop().whenHeld(new StartEndCommand(wheelOfFortune::extendWOFManipulator, wheelOfFortune::retractWOFManipulator));
		stratComInterface.getSC1().whenHeld(new StartEndCommand(wheelOfFortune::spinCounterclockwise, wheelOfFortune::stopSpin, wheelOfFortune));//torque vector up
		stratComInterface.getSC3().whenHeld(new StartEndCommand(wheelOfFortune::spinClockwise, wheelOfFortune::stopSpin, wheelOfFortune));//torque vector down
		stratComInterface.getSC2().whenHeld(new RunEndCommand(wheelOfFortune::spinThreeTimes, wheelOfFortune::stopSpin, wheelOfFortune));
	}

	private void configTestingCommands() {
//		stratComInterface.getSC4().whenHeld(new StartEndCommand(() -> spindexer.setSpinRPM(30, Spindexer.SpinnerDirection.Clockwise), spindexer::stopSpin, spindexer));
//		stratComInterface.getSC1().whenHeld(new StartEndCommand(() ->  spindexer.setSpinRPM(30, Spindexer.SpinnerDirection.CounterClockwise), spindexer::stopSpin, spindexer));

//		stratComInterface.getDoubleToggleUp().whenHeld(new StartEndCommand(superStructure::intake, superStructure::idleIntake, superStructure, spindexer));
//		stratComInterface.getDoubleToggleDown().whenHeld(new StartEndCommand(superStructure::outtake, superStructure::idleIntake, superStructure, spindexer));

//		stratComInterface.getDoubleToggleDown().whenHeld(new RunEndCommand(() -> intake.setPower(stratComInterface.getLeftSlider()), superStructure::dumbIntakeIdle, superStructure, intake));

//		stratComInterface.getSingleToggle().whenHeld(new RunEndCommand(() -> shooter.setDumbRPM(OscarMath.clipMap(stratComInterface.getRightSlider(), -1, 1, 0, 5000)), shooter::idle));
//		stratComInterface.getSingleToggle().whenHeld(new RunEndCommand(() -> shooter.setFeedRPM(OscarMath.clipMap(stratComInterface.getLeftSlider(), -1, 1, 0, 4000)), shooter::idle));

		stratComInterface.getArcadeWhiteLeft().whenHeld(new RunEndCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.TARGETING),
				() -> superStructure.setState(SuperStructure.SuperstructureState.IDLE)));
		stratComInterface.getArcadeWhiteRight().whenHeld(new RunEndCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.SHOOTING),
				() -> superStructure.setState(SuperStructure.SuperstructureState.IDLE)));
		stratComInterface.getArcadeBlackLeft().whenHeld(new RunEndCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.INTAKE),
				() -> superStructure.setState(SuperStructure.SuperstructureState.IDLE)));
		stratComInterface.getArcadeBlackRight().whenHeld(new RunEndCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.IDLE),
				() -> superStructure.setState(SuperStructure.SuperstructureState.IDLE)));

		stratComInterface.getDoubleToggleUp().whenHeld(new RunEndCommand(() -> superStructure.setShootingState(SuperStructure.ShootingState.FIRING),
				() -> superStructure.setShootingState(SuperStructure.ShootingState.PREPARE)));

		stratComInterface.getKeySwitch().whenHeld(new RunEndCommand(() -> superStructure.setShooterParametersSlider(stratComInterface.getLeftSlider(), stratComInterface.getRightSlider()),
				() -> superStructure.setShooterParametersdefault()));

//		stratComInterface.getArcadeWhiteLeft().whileHeld(superStructure.prepareShootCommand);
//		stratComInterface.getArcadeWhiteRight().whileHeld(superStructure.shootCommand);

		stratComInterface.getSingleToggle().whenHeld(new RunEndCommand(() -> climber.adjustHook(stratComInterface.getLeftSlider()), climber::stopExtend));
		stratComInterface.getSingleToggle().whenReleased(new InstantCommand(climber::retractHook));

		stratComInterface.getSCPlus().whileHeld(new ClimbGroup(climber, true));
		stratComInterface.getSCPlus().whenReleased(new InstantCommand(climber::lockClimb));

		stratComInterface.getSCMinus().whileHeld(new ClimbGroup(climber, false));
		stratComInterface.getSCMinus().whenReleased(new InstantCommand(climber::lockClimb));


	}
}

