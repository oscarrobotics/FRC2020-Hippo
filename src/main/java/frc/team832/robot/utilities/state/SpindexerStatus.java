package frc.team832.robot.utilities.state;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.driverstation.dashboard.DashboardWidget;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.impl.SmartMCAttachedPDPSlot;
import frc.team832.lib.power.monitoring.StallDetector;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;
import frc.team832.robot.subsystems.Spindexer;
import frc.team832.robot.subsystems.SuperStructure;
import frc.team832.robot.utilities.positions.BallPosition;

public class SpindexerStatus implements DashboardUpdatable {
    private final Spindexer spindexer;
    private final CANSparkMax spinMotor;
    private final GrouchPDP pdp;
    private final SmartMCAttachedPDPSlot spinSlot;
    private final StallDetector spinStall;

    private boolean[] ballPositions = new boolean[5];
    private double absoluteRotations = 0;
    public SpindexerState state;
    private Spindexer.SpinnerDirection spinDirection;

    NetworkTableEntry ballSlot0, ballSlot1, ballSlot2, ballSlot3, ballSlot4, dashboard_state, dashboard_hallEffect;

    public SpindexerStatus(GrouchPDP pdp, Spindexer spindexer, CANSparkMax spinMotor) {

        this.spindexer = spindexer;
        this.spinMotor = spinMotor;
        this.pdp = pdp;

        spinSlot = pdp.addDevice(Constants.SpindexerValues.SPIN_MOTOR_PDP_SLOT, spinMotor);
        spinStall = new StallDetector(spinSlot);
        spinStall.setMinStallMillis(500);
        spinStall.setStallCurrent(30);

        ballSlot0 = DashboardManager.addTabItem(this, "Slot 0", false, DashboardWidget.BooleanBox);
        ballSlot1 = DashboardManager.addTabItem(this, "Slot 1", false, DashboardWidget.BooleanBox);
        ballSlot2 = DashboardManager.addTabItem(this, "Slot 2", false, DashboardWidget.BooleanBox);
        ballSlot3 = DashboardManager.addTabItem(this, "Slot 3", false, DashboardWidget.BooleanBox);
        ballSlot4 = DashboardManager.addTabItem(this, "Slot 4", false, DashboardWidget.BooleanBox);
        dashboard_state = DashboardManager.addTabItem(this, "State", "Default");
        dashboard_hallEffect = DashboardManager.addTabItem(this, "Hall Effect", false, DashboardWidget.BooleanBox);
    }

    public void update(boolean isOverSlot) {
        boolean currentSlotHasBall = spindexer.getBallSensor() && isOverSlot;
        ballPositions[spindexer.getNearestBallPosition().slotNumber] = currentSlotHasBall;

        if (isFull())
            state = SpindexerState.FULL;
        else if (!isEmpty())
            state = SpindexerState.FILLING;
        else
            state = SpindexerState.EMPTY;

        spinStall.updateStallStatus();

//        if (spindexer.getHallEffect()) {
//           onHallEffect();
//        }

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

    @Override
    public String getDashboardTabName() {
        return "Spindexer Status";
    }

    @Override
    public void updateDashboardData() {
        ballSlot0.setBoolean(getSlot(0));
        ballSlot1.setBoolean(getSlot(1));
        ballSlot2.setBoolean(getSlot(2));
        ballSlot3.setBoolean(getSlot(3));
        ballSlot4.setBoolean(getSlot(4));
        dashboard_state.setString(state.toString());
        dashboard_hallEffect.setBoolean(spindexer.getHallEffect());
    }

    public enum SpindexerState {
        FULL,
        FILLING,
        EMPTY
    }
}
