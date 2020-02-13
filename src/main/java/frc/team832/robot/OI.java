package frc.team832.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.team832.lib.driverinput.controllers.*;
import frc.team832.lib.driverinput.oi.DriverOI;
import frc.team832.lib.driverinput.oi.SticksDriverOI;
import frc.team832.lib.driverinput.oi.XboxDriverOI;
import frc.team832.lib.util.OscarMath;

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
		stratComInterface.getSCPlus().whileHeld(new StartEndCommand(climber::climbUp, climber::stopClimb, Robot.climber));
		stratComInterface.getSCMinus().whileHeld(new StartEndCommand(climber::climbDown, climber::stopClimb, Robot.climber));

//		stratComInterface.getSCSideMid().whileHeld(new StartEndCommand(() -> superStructure.setMode(SuperStructure.SuperStructureMode.INTAKING), () -> superStructure.setMode(SuperStructure.SuperStructureMode.IDLELAST), superStructure, intake, spindexer));
//		stratComInterface.getSCSideBot().whileHeld(new StartEndCommand(() -> superStructure.setMode(SuperStructure.SuperStructureMode.OUTTAKING), () -> superStructure.setMode(SuperStructure.SuperStructureMode.IDLELAST), superStructure, intake, spindexer));

//		stratComInterface.getArcadeBlackRight().whenPressed(new PrepareShooter(superStructure, pneumatics, shooter, spindexer));
//		stratComInterface.getArcadeBlackLeft().whileHeld(new ShootCommandGroup(superStructure, pneumatics, shooter, spindexer));

//		stratComInterface.getSC2().whileHeld(new StartEndCommand(pneumatics::extendWOFManipulator, pneumatics::retractWOFManipulator, Robot.pneumatics));
//		stratComInterface.getSC1().whileHeld(new StartEndCommand(wheelOfFortune::spinCounterclockwise, wheelOfFortune::stopSpin, Robot.wheelOfFortune));
//		stratComInterface.getSC3().whileHeld(new StartEndCommand(wheelOfFortune::spinClockwise, wheelOfFortune::stopSpin, Robot.wheelOfFortune));
//		stratComInterface.getSC6().whenPressed(new InstantCommand(wheelOfFortune::spinThreeTimes, Robot.wheelOfFortune));


		//Dumb testing commands
		stratComInterface.getSC4().whenHeld(new StartEndCommand(() -> spindexer.spinClockwise(0.25), spindexer::stopSpin, spindexer));//All 3 of these could be being set to idle by superstructure every loop
		stratComInterface.getSC5().whenHeld(new StartEndCommand(shooter::spin, shooter::stopAll, shooter));
		stratComInterface.getSCSideMid().whenHeld(new StartEndCommand(() -> intake.intake(0.9), intake::stop, intake));

		stratComInterface.getSingleToggle().whileHeld(new InstantCommand(() -> shooter.setPower(OscarMath.clipMap(stratComInterface.getRightSlider(), -1, 1, 0, 0.65))));
		stratComInterface.getSingleToggle().whileHeld(new InstantCommand(() -> shooter.feed(OscarMath.clipMap(stratComInterface.getLeftSlider(), -1, 1, 0, 0.7))));


//		stratComInterface.getSingleToggle().whileHeld(new InstantCommand(() -> shooter.setHeadingRotation(stratComInterface.getLeftSlider())));
//		stratComInterface.getDoubleToggleUp().whileHeld(new InstantCommand(() -> shooter.setDumbRPM(OscarMath.clipMap(stratComInterface.getRightSlider(), -1, 1, 0, 5000))));
//		stratComInterface.getDoubleToggleDown().whileHeld(new InstantCommand(() -> shooter.setExitAngle(OscarMath.clipMap(stratComInterface.getRightSlider(), -1, 1, 20, 70))));
	}
}
