package frc.team832.robot.subsystems;

import frc.team832.lib.power.GrouchPDP;
import org.junit.jupiter.api.*;

import static org.junit.jupiter.api.Assertions.*;

public class SuperStructureTests {
	private static SuperStructure superStructure;
	private static Shooter shooter;
	private static Intake intake;
	private static Spindexer spindexer;
	private static Turret turret;
	private static Vision vision;

	@BeforeAll
	public static void init() {
		var pdp = new GrouchPDP(0);
		shooter = new Shooter(pdp);
		intake = new Intake(pdp);
		spindexer = new Spindexer(pdp);
		turret = new Turret(pdp);
		vision = new Vision();
		superStructure = new SuperStructure(intake, shooter, spindexer, turret, vision);
	}

	@Test
	public void isShooterReady(){
		assertTrue(shootReadyTest(1000.0, 997.7, 5000.0, 4978.3, .8), "Everything at speed");
	}

	public boolean shootReadyTest(double feederActual, double feederTarget, double shooterTarget, double shooterActual, double trackingOffset) {
		boolean feederReady, shooterReady, trackingReady;

		feederReady = Math.abs(feederTarget - feederActual) <= 50;
		shooterReady = Math.abs(shooterTarget - shooterActual) <= 100;
		trackingReady = trackingOffset < 1.0;

		return  feederReady && shooterReady && trackingReady;
	}

	@AfterAll
	public static void end() {
		shooter.close();
		turret.close();
	}
}
