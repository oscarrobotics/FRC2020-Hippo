package frc.team832.robot.utilities.state;

import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.impl.SmartMCAttachedPDPSlot;
import frc.team832.lib.power.monitoring.StallDetector;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;
import frc.team832.robot.subsystems.Spindexer;

public class SpindexerStatus {
    private final Spindexer spindexer;
    private final CANSparkMax spinMotor;
    private final GrouchPDP pdp;
    private final SmartMCAttachedPDPSlot spinSlot;
    private final StallDetector spinStall;

    private boolean[] ballPositions = new boolean[5];
    private double absoluteRotations = 0;
    public SpindexerState state = SpindexerState.EMPTY;
    private Spindexer.SpinnerDirection spinDirection;

    public SpindexerStatus(GrouchPDP pdp, Spindexer spindexer, CANSparkMax spinMotor) {

        this.spindexer = spindexer;
        this.spinMotor = spinMotor;
        this.pdp = pdp;

        spinSlot = pdp.addDevice(Constants.SpindexerValues.SPIN_MOTOR_PDP_SLOT, spinMotor);
        spinStall = new StallDetector(spinSlot);
        spinStall.setMinStallMillis(500);
        spinStall.setStallCurrent(30);

    }

    public void update() {
        boolean currentSlotHasBall = spindexer.isBall() && spindexer.isOverSlot();
        ballPositions[spindexer.getNearestBallPosition().slotNumber] = currentSlotHasBall;

        if (isFull())
            state = SpindexerState.FULL;
        else if (!isEmpty())
            state = SpindexerState.FILLING;
        else
            state = SpindexerState.EMPTY;

        spinStall.updateStallStatus();

        if (spindexer.getHallEffect()) {
           onHallEffect();
        }

        spinDirection = Math.signum(spinMotor.getSensorVelocity()) == 1 ? Spindexer.SpinnerDirection.Clockwise : Spindexer.SpinnerDirection.CounterClockwise;
    }

    public void onHallEffect() {
        spindexer.zeroSpindexer();
        if (spindexer.getSpinnerDirection() == Spindexer.SpinnerDirection.Clockwise) absoluteRotations++;
        else absoluteRotations--;
    }

    public boolean getSlot(int pos) {
        pos = OscarMath.clip(pos, 0, 4);
        return ballPositions[pos];
    }

    public boolean isFull() { return getSlot(0) && getSlot(1) && getSlot(2) && getSlot(3) && getSlot(4); }

    public boolean isEmpty() {
        return !getSlot(0) && !getSlot(1) && !getSlot(2) && !getSlot(3) && !getSlot(4);
    }

    public boolean isStalling() {
        return spinStall.getStallStatus().isStalled;
    }

    public double getBallNumber() {
        double count = 0;
        for (int i = 0; i < 5; i++) {
            if (ballPositions[i])
                count++;
        }
        return count;
    }

    public double getAbsoluteRotations() {
        return absoluteRotations + spindexer.getRelativeRotations();
    }

    public int getFirstEmpty(){
        for(int i = 0; i < 5; i++) {
            if(!ballPositions[i]) {
                return i;
            }
        }
        return 0;
    }

    public SpindexerState getState() {
        return state;
    }

    public boolean[] getBooleanList() { return ballPositions; }

    public Spindexer.SpinnerDirection getSpinDirection() {
        return spinDirection;
    }

    public enum SpindexerState {
        FULL,
        FILLING,
        EMPTY
    }
}
