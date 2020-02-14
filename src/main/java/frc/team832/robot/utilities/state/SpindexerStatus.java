package frc.team832.robot.utilities.state;

import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.impl.SmartMCAttachedPDPSlot;
import frc.team832.lib.power.monitoring.StallDetector;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;
import frc.team832.robot.subsystems.Spindexer;

import java.util.ArrayList;
import java.util.List;

public class SpindexerStatus {
    private final Spindexer spindexer;
    private final CANSparkMax spinMotor;
    private final GrouchPDP pdp;
    private final SmartMCAttachedPDPSlot spinSlot;
    private final StallDetector spinStall;

    private List<Boolean> ballPositions = new ArrayList<>();
    private double absoluteRotations = 0;
    public SpindexerState state;
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

    public void update(List<Boolean> sensorData, boolean hallEffect) {
//        this.ballPositions = sensorData;
//        if (isFull())
//            state = SpindexerState.FULL;
//        else if (!isEmpty())
//            state = SpindexerState.FILLING;
//        else
//            state = SpindexerState.EMPTY;
//
//        spinStall.updateStallStatus();
//
//        if (hallEffect) {
//            spindexer.zeroSpindexer();
//            if (spindexer.getSpinnerDirection() == Spindexer.SpinnerDirection.Clockwise) absoluteRotations++;
//            else absoluteRotations--;
//        }
//
//        spinDirection = Math.signum(spinMotor.getSensorVelocity()) == 1 ? Spindexer.SpinnerDirection.Clockwise : Spindexer.SpinnerDirection.CounterClockwise;
    }

    public boolean getPosition(int pos) {
        pos = OscarMath.clip(pos, 1, 5) - 1;
        return ballPositions.get(pos);
    }

    public boolean isFull() {
        return getPosition(1) && getPosition(2) && getPosition(3) && getPosition(4) && getPosition(5);
    }

    public boolean isEmpty() {
        return !getPosition(1) && !getPosition(2) && !getPosition(3) && !getPosition(4) && !getPosition(5);
    }

    public boolean isStalling() {
        return spinStall.getStallStatus().isStalled;
    }

    public double getBallNumber() {
        double count = 0;
        for (int i = 0; i < 5; i++) {
            if (ballPositions.get(i))
                count++;
        }
        return count;
    }

    public double getAbsoluteRotations() {
        return absoluteRotations + spinMotor.getSensorPosition();
    }

    public int getFirstEmpty(){
        for(int i = 0; i < 5; i++) {
            if(!getBooleanList().get(i)){
                return i;
            }
        }
        return 0;
    }

    public SpindexerState getState() {
        return state;
    }

    public List<Boolean> getBooleanList() { return ballPositions; }

    public Spindexer.SpinnerDirection getSpinDirection() {
        return spinDirection;
    }

    public enum SpindexerState {
        FULL,
        FILLING,
        EMPTY
    }
}
