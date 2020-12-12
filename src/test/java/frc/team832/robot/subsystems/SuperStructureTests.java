package frc.team832.robot.subsystems;

import edu.wpi.first.hal.HAL;
import frc.team832.lib.power.GrouchPDP;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class SuperStructureTests {
	static SuperStructure superStructure;
	static Shooter shooter;
	static Intake intake;
	static Spindexer spindexer;
	static Turret turret;
	static Vision vision;

	@Before
	public void init() {
		shooter = new Shooter(new GrouchPDP(0));
		intake = new Intake(new GrouchPDP(0));
		spindexer = new Spindexer(new GrouchPDP(0));
		turret = new Turret(new GrouchPDP(0));
		vision = new Vision();
		superStructure = new SuperStructure(intake, shooter, spindexer, turret, vision);
	}

	@Test
	public void isShooterReady(){
		assertTrue("Everything at speed", shootReadyTest(1000.0, 997.7, 5000.0, 4978.3, .8));
	}

	public boolean shootReadyTest(double feederActual, double feederTarget, double shooterTarget, double shooterActual, double trackingOffset) {
		boolean feederReady, shooterReady, trackingReady;

		feederReady = Math.abs(feederTarget - feederActual) <= 50;
		shooterReady = Math.abs(shooterTarget - shooterActual) <= 100;
		trackingReady = trackingOffset < 1.0;

		return  feederReady && shooterReady && trackingReady;
	}

	@After
	public void end() {
		HAL.shutdown();
	}
}
