package frc.team832.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.team832.lib.driverinput.controllers.*;

public class OI {

	//Creates the drivePad object of XboxController360 class
	public static final Xbox360Controller drivePad = new Xbox360Controller(0);
	//Creates the stratComInterface of the StratComInterface Class
	public static final StratComInterface stratComInterface = new StratComInterface(1);

	public static final Joystick leftDriveStick = new Attack3(2);
	public static final Joystick rightDriveStick = new Extreme3DPro(3);

	//Commands: drivePad
//        drivePad.rightStickPress.whenPressed(new InstantCommand(() -> vision.setDriverMode(false))).whenReleased(new InstantCommand(() -> vision.setDriverMode(true)));

	//Commands: stratComInterface

//	var keySwitchCommand = new RunCommand(superStructure::moveManual, fourbar, elevator, superStructure);
//        stratComInterface.getKeySwitch().whileActiveContinuous(keySwitchCommand);

//        drivePad.aButton.whenPressed(new AutonomousHatchScore(Paths.RightHab_RightFrontRocket, SuperStructure.SuperStructurePosition.CARGOSHIP_HATCH, drivetrain, superStructure, elevator, fourbar, intake));
//        drivePad.aButton.whenPressed(new RamseteCommand(Paths.Test_Three_Meters_Forward, drivetrain::getLatestPose2d, new RamseteController(2, 0.7), DRIVE_KINEMATICS, drivetrain::consumeWheelSpeeds, drivetrain));


}