package frc.team832.robot.subsystems;

import edu.wpi.first.hal.HAL;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.robot.Constants;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class TurretTest {
    static Turret turret;

    @Before
    public void init() {
        turret = new Turret(new GrouchPDP(0));
    }

    @Test
    public void testVisionSafety(){
        int safeTestingRange = Constants.TurretValues.VisionTargetingRange / 2;

        for (int i = -safeTestingRange; i < safeTestingRange; i++) {
            var safeTargetPosition = turret.calculateSafePosition(true, i);
            boolean wasSafe = i == safeTargetPosition;
            assertTrue("Safety failed! TargetDeg:" + i + ", SafeDeg: " + safeTargetPosition, wasSafe);
        }

        int badTestingRangeStart = -(Constants.TurretValues.VisionTargetingRange + 1);
        int badTestingRangeEnd = -badTestingRangeStart;

        for (int i = badTestingRangeStart; i < badTestingRangeEnd; i++) {
            if (!(i <= safeTestingRange && i >= -safeTestingRange)) {
                var safeTargetPosition = turret.calculateSafePosition(true, i);
                boolean wasSafe = i == safeTargetPosition;

                assertFalse("Safety unexpectedly passed! TargetDeg:" + i + ", SafeDeg: " + safeTargetPosition, wasSafe);
            }
        }
    }

    @After
    public void end() {
        turret = null;
        HAL.shutdown();
    }
}
