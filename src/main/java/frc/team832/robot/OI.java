package frc.team832.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunEndCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.team832.lib.driverinput.controllers.*;
import frc.team832.lib.driverinput.oi.DriverOI;
import frc.team832.lib.driverinput.oi.SticksDriverOI;
import frc.team832.lib.driverinput.oi.XboxDriverOI;
import frc.team832.lib.sensors.digital.HallEffect;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.commands.PrepareShooter;
import frc.team832.robot.commands.ShootCommandGroup;
import frc.team832.robot.subsystems.*;

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
//		stratComInterface.getSingleToggle().whenPressed(new InstantCommand(climber::extendHook));
//		stratComInterface.getSingleToggle().whenHeld(new RunEndCommand(() -> climber.adjustHook(stratComInterface.getLeftSlider()), climber::stopExtend));
//		stratComInterface.getSingleToggle().whenReleased(new InstantCommand(climber::retractHook));

//		stratComInterface.getSCPlus().whenHeld(new StartEndCommand(climber::climbUp, climber::stopClimb, climber));
//		stratComInterface.getSCMinus().whenHeld(new StartEndCommand(climber::climbDown, climber::stopClimb, climber));


//		stratComInterface.getArcadeWhiteLeft().whenPressed(new PrepareShooter(superStructure, pneumatics, shooter, spindexer));
//		stratComInterface.getArcadeBlackLeft().whenPressed(new ShootCommandGroup(superStructure, pneumatics, shooter, spindexer));


//		stratComInterface.getArcadeWhiteRight().whenHeld(new StartEndCommand(superStructure::intake, superStructure::idleIntake, superStructure, intake, spindexer));
//		stratComInterface.getArcadeBlackRight().whenHeld(new StartEndCommand(superStructure::outtake, superStructure::idleIntake, superStructure, intake, spindexer));

//		stratComInterface.getSCSideBot().whenHeld(new StartEndCommand(pneumatics::extendIntake, pneumatics::retractIntake, superStructure, intake, spindexer));


//		stratComInterface.getSCSideTop().whenHeld(new StartEndCommand(pneumatics::extendWOFManipulator, pneumatics::retractWOFManipulator, pneumatics));
//		stratComInterface.getSC1().whenHeld(new StartEndCommand(wheelOfFortune::spinCounterclockwise, wheelOfFortune::stopSpin, wheelOfFortune));
//		stratComInterface.getSC3().whenHeld(new StartEndCommand(wheelOfFortune::spinClockwise, wheelOfFortune::stopSpin, wheelOfFortune));
//		stratComInterface.getSC2().whenHeld(new RunEndCommand(wheelOfFortune::spinThreeTimes, wheelOfFortune::stopSpin, wheelOfFortune));

		//Dumb testing commands
		stratComInterface.getSC4().whenHeld(new StartEndCommand(() -> spindexer.spinClockwise(0.5), spindexer::stopSpin, spindexer));
		stratComInterface.getSC1().whenHeld(new StartEndCommand(() -> spindexer.spinCounterclockwise(0.5), spindexer::stopSpin, spindexer));

		stratComInterface.getSC5().whenHeld(new StartEndCommand(shooter::spin, shooter::stopAll, shooter));
        stratComInterface.getSC2().whenHeld(new StartEndCommand(shooter::otherSpin, shooter::stopAll, shooter));

		stratComInterface.getDoubleToggleUp().whenHeld(new RunEndCommand(() -> superStructure.dumbIntake(stratComInterface.getLeftSlider()), superStructure::dumbIntakeIdle, superStructure, intake));
		stratComInterface.getSCSideBot().whenHeld(new StartEndCommand(() -> intake.outtake(0.5), intake::stop, intake));

		stratComInterface.getSingleToggle().whenHeld(new RunEndCommand(() -> shooter.setDumbRPM(OscarMath.clipMap(stratComInterface.getRightSlider(), -1, 1, 0, 5000)), shooter::idle));
		stratComInterface.getSingleToggle().whenHeld(new RunEndCommand(() -> shooter.setDumbFeedRPM(OscarMath.clipMap(stratComInterface.getLeftSlider(), -1, 1, 0, 4000)), shooter::idle));

		stratComInterface.getArcadeBlackRight().whileHeld(new StartEndCommand(() -> shooter.setHood(0.5), shooter::idleHood));
		stratComInterface.getArcadeWhiteRight().whileHeld(new StartEndCommand(() -> shooter.setHood(-0.5), shooter::idleHood));

		stratComInterface.getArcadeBlackRight().whenPressed(new InstantCommand(() -> shooter.setTurretPosition(Constants.ShooterValues.PracticeTurretForwardPosition), shooter));
		stratComInterface.getArcadeWhiteRight().whenPressed(new InstantCommand(() -> shooter.setTurretPosition(.08), shooter));
		stratComInterface.getArcadeWhiteLeft().whenPressed(new InstantCommand(() -> shooter.setTurretPosition(.58), shooter));


//		stratComInterface.getDoubleToggleDown().whileHeld(new InstantCommand(() -> shooter.setExitAngle(OscarMath.clipMap(stratComInterface.getRightSlider(), -1, 1, 20, 70))));
	}
}
