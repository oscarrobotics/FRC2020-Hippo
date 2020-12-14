package frc.team832.robot.subsystems;

import frc.team832.lib.power.GrouchPDP;
import org.junit.jupiter.api.*;

import static org.junit.jupiter.api.Assertions.*;


public class SpindexerTests {
    private static SuperStructure superStructure;
    private static Shooter shooter;
    private static Intake intake;
    private static Spindexer spindexer;
    private static Turret turret;
    private static Vision vision;

    @BeforeAll
    public static void init() {
        shooter = new Shooter(new GrouchPDP(0));
        intake = new Intake(new GrouchPDP(0));
        spindexer = new Spindexer(new GrouchPDP(0));
        turret = new Turret(new GrouchPDP(0), spindexer);
        vision = new Vision();
        superStructure = new SuperStructure(intake, shooter, spindexer, turret, vision);
    }

    @Test
    public void isSpindexerReadyTest(){
        boolean valid = superStructure.isSpindexerReadyShoot(.2, .23);
        boolean invalid = superStructure.isSpindexerReadyShoot(.2, .3);

        assertTrue(valid, "InRange \"double\" failed on valid!");
        assertFalse(invalid, "InRange \"double\" failed on invalid!");
    }

    @AfterAll
    public static void end() {
        turret.close();
        shooter.close();
    }
}
