package frc.team832.robot.subsystems;

import frc.team832.lib.power.GrouchPDP;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class ShooterTests {

    static Shooter shooter;

    @Before
    public void init() {
        shooter = new Shooter(new GrouchPDP(0), new Vision(new Drivetrain(new GrouchPDP(0))));
    }

    @Test
    public void isTurretSafeTest() {
        double turretTarget = 0.06;
        double turretPosition = 0.05;

        boolean valid = shooter.isTurretSafe(turretTarget, turretPosition) == Shooter.TurretSafetyState.FarLeft;
        boolean invalid = shooter.isTurretSafe(turretTarget, turretPosition) == Shooter.TurretSafetyState.Safe;

        assertTrue("InRange \"double\" failed on valid!", valid);
        assertFalse("InRange \"double\" failed on invalid!", invalid);
    }

    @Test
    public void visionTest() {

    }
}
