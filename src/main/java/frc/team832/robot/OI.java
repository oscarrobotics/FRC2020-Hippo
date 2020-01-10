package frc.team832.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.team832.lib.driverinput.controllers.*;
import frc.team832.lib.driverinput.oi.DriverOI;
import frc.team832.lib.driverinput.oi.SticksDriverOI;

public class OI {

	public final DriverOI driverOI;
	public static final StratComInterface stratComInterface = new StratComInterface(1);

	public final Attack3 leftStick;
	public final Extreme3DPro rightStick;

	public OI() {
//        driverOI = new XboxDriverOI();
		driverOI = new SticksDriverOI();

		leftStick = ((SticksDriverOI)driverOI).leftStick;
		rightStick = ((SticksDriverOI)driverOI).rightStick;

		// do commands here

	}

}