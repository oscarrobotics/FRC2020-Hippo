package frc.team832.robot.subsystems;

import frc.team832.lib.power.GrouchPDP;
import frc.team832.robot.Constants;
import org.junit.jupiter.api.*;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class TurretTest {
    private static Turret turret;

    @BeforeAll
    public void init() {
        var pdp = new GrouchPDP(0);
        turret = new Turret(new GrouchPDP(0), new Spindexer(pdp));
    }

    @Test
    public void testVisionSafety(){
        int safeTestingRange = Constants.TurretValues.VisionTargetingRange / 2;

        for (int i = -safeTestingRange; i < safeTestingRange; i++) {
            var safeTargetPosition = turret.calculateSafePosition(true, i);
            boolean wasSafe = i == safeTargetPosition;
            assertTrue(wasSafe, "Safety failed! TargetDeg:" + i + ", SafeDeg: " + safeTargetPosition);
        }

        int badTestingRangeStart = -(Constants.TurretValues.VisionTargetingRange + 1);
        int badTestingRangeEnd = -badTestingRangeStart;

        for (int i = badTestingRangeStart; i < badTestingRangeEnd; i++) {
            if (!(i <= safeTestingRange && i >= -safeTestingRange)) {
                var safeTargetPosition = turret.calculateSafePosition(true, i);
                boolean wasSafe = i == safeTargetPosition;

                assertFalse(wasSafe, "Safety unexpectedly passed! TargetDeg:" + i + ", SafeDeg: " + safeTargetPosition);
            }
        }
    }

    @AfterAll
    public static void end() {
        turret.close();
    }
}
