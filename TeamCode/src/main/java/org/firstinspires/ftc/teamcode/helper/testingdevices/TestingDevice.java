package org.firstinspires.ftc.teamcode.helper.testingdevices;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * A hardware device to use for testing.
 * This class is meant to be extended by other classes to handle specific types of hardware device.
 */
public abstract class TestingDevice {
    protected final String hardwareName;
    protected HardwareDevice hardwareDevice = null;

    protected boolean frozen = false;
    protected final Object[] states = new Object[Constants.TESTING_SAVE_STATE_SLOTS];

    // When move() or offset() runs, it sets its value. Both values are unset after both methods have run.
    // When one of the methods runs, it checks other's value. If there's no value, that means it hasn't run since last time.
    protected Double moveAmount = null;
    protected Double offsetAmount = null;
    protected boolean isInMoveMode = true;

    public TestingDevice(String hardwareName) {
        this.hardwareName = hardwareName;
    }

    public String getHardwareName() {
        return hardwareName;
    }

    public HardwareDevice getHardwareDevice() {
        return hardwareDevice;
    }

    public boolean isActive() {
        return hardwareDevice != null;
    }

    public void deactivate() {
        if (isActive()) hardwareDevice.close();
        hardwareDevice = null;
        freeze(false);
    }

    public void move(double amount) {
        if (frozen) return;
        moveAmount = amount;
        decideAndMove();
    }

    public void offset(double amount) {
        if (frozen) return;
        offsetAmount = amount;
        decideAndMove();
    }

    public void changeToMove() {
        // To be optionally overridden by subclasses
    }

    public void changeToOffset() {
        // To be optionally overridden by subclasses
    }

    /**
     * Determine whether to move or offset, then do so by calling {@link #moveDirectly(double)} or {@link #offsetDirectly(double)}.
     * <p>If the greater absolute value between the move amount and the offset amount is greater than
     * {@code modeChangeThreshold}, then it becomes the new mode of movement.
     * If neither is over the threshold, then it stays on its current mode.</p>
     * <p>If either {@code moveAmount} or {@code offsetAmount} isn't set, this does nothing.</p>
     * <p>The purpose of this is for cases where there's a switch between moving and offsetting that
     * needs to be handled specially (i.e. motors, which need to switch between run modes).</p>
     */
    private void decideAndMove() {
        // Guards
        if (moveAmount == null || offsetAmount == null) return;

        // Mode change
        if (Math.abs(moveAmount) > Math.abs(offsetAmount)) {
            if (Math.abs(moveAmount) > Constants.TESTING_MODE_CHANGE_THRESHOLD) {
                isInMoveMode = true;
                changeToMove();
            }
        } else {
            if (Math.abs(offsetAmount) > Constants.TESTING_MODE_CHANGE_THRESHOLD) {
                isInMoveMode = false;
                changeToOffset();
            }
        }

        // Action
        if (isInMoveMode) {
            moveDirectly(moveAmount);
        } else {
            offsetDirectly(offsetAmount);
        }

        // Reset
        moveAmount = null;
        offsetAmount = null;
    }

    public void zero() {
        // To be optionally overridden by subclasses
    }

    public void freeze(boolean doFreeze) {
        if (frozen == doFreeze) return;
        frozen = doFreeze;
        if (frozen) {
            startFreeze();
        } else {
            endFreeze();
        }
    }

    public boolean isFrozen() {
        return frozen;
    }

    protected void startFreeze() {
        // To be optionally overridden by subclasses
    }

    protected void endFreeze() {
        // To be optionally overridden by subclasses
    }

    public void storeState(int slot) {
        states[slot] = getState();
    }

    public void loadState(int slot) {
        if (states[slot] != null) {
            freeze(false);
            applyState(states[slot]);
            freeze(true);
        }
    }

    public abstract void activate(HardwareMap hardwareMap);
    protected abstract void moveDirectly(double amount);
    protected abstract void offsetDirectly(double amount);
    protected abstract Object getState();
    protected abstract void applyState(Object state);
    public abstract void addStatusToTelemetry(Telemetry telemetry);
}
