package frc.team832.robot.accesories;

import frc.team832.lib.util.OscarMath;

public class SpindexerStatus {
    private boolean[] ballPositions;
    public SpindexerState state;

    public void update(boolean[] sensorData) {
        ballPositions = sensorData;
        if (isFull())
            state = SpindexerState.FULL;
        else if (!isEmpty())
            state = SpindexerState.FILLING;
        else
            state = SpindexerState.EMPTY;
    }

    public boolean getPosition(int pos) {
        pos = OscarMath.clip(pos, 1, 5) - 1;
        return ballPositions[pos];
    }

    public boolean isFull() {
        return getPosition(1) && getPosition(2) && getPosition(3) && getPosition(4) && getPosition(5);
    }

    public boolean isEmpty() {
        return !getPosition(1) && !getPosition(2) && !getPosition(3) && !getPosition(4) && !getPosition(5);
    }

    public SpindexerState getState() {
        return state;
    }

    public boolean[] getStateArray() {
        return ballPositions;
    }

    public enum SpindexerState {
        FULL,
        FILLING,
        EMPTY
    }
}
