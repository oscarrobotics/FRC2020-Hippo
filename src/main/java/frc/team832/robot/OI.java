package frc.team832.robot;

import frc.team832.lib.driverinput.controllers.*;
import frc.team832.lib.driverinput.oi.DriverOI;
import frc.team832.lib.driverinput.oi.SticksDriverOI;
import frc.team832.lib.driverinput.oi.XboxDriverOI;
import frc.team832.robot.commands.IntakePowerCell;

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

		// do commands here
		stratComInterface.getArcadeWhiteLeft().whileHeld(new IntakePowerCell(Robot.intake, Robot.spindexer));
	}

}