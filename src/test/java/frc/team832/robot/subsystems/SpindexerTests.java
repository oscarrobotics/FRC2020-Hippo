package frc.team832.robot.subsystems;

import frc.team832.lib.power.GrouchPDP;
import frc.team832.robot.Constants;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class SpindexerTests {
    static SuperStructure superStructure;
    static Shooter shooter;
    static Intake intake;
    static Spindexer spindexer;

    @Before
    public void init() {
        shooter = new Shooter(new GrouchPDP(0), new Vision(new Drivetrain(new GrouchPDP(0))));
        intake = new Intake(new GrouchPDP(0));
        spindexer = new Spindexer(new GrouchPDP(0));
        superStructure = new SuperStructure(intake, shooter, spindexer);

    }

    @Test
    public void isSpindexerReadyTest(){
        boolean valid = superStructure.isSpindexerReadyShoot(.2, .23);
        boolean invalid = superStructure.isSpindexerReadyShoot(.2, .3);

        assertTrue("InRange \"double\" failed on valid!", valid);
        assertFalse("InRange \"double\" failed on invalid!", invalid);

    }

}
