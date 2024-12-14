package org.firstinspires.ftc.teamcode.helper.testingdevices;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Calculations;
import org.firstinspires.ftc.teamcode.Constants;

public class TestingDcMotor extends TestingDevice {
    public TestingDcMotor(String hardwareName) {
        super(hardwareName);
    }

    @Override
    public DcMotor getHardwareDevice() {
        return (DcMotor) hardwareDevice;
    }

    @Override
    public void activate(HardwareMap hardwareMap) {
        isInRunToPositionMode = false;
        hardwareDevice = hardwareMap.get(DcMotor.class, hardwareName);
        getHardwareDevice().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        getHardwareDevice().setDirection(DcMotor.Direction.FORWARD);
        getHardwareDevice().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        getHardwareDevice().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    protected void moveDirectly(double amount) {
        if (!isActive()) return;
        getHardwareDevice().setPower(amount);
    }

    @Override
    protected void offsetDirectly(double amount) {
        if (!isActive()) return;
        getHardwareDevice().setTargetPosition(getHardwareDevice().getTargetPosition() + (int) (amount * Constants.TESTING_DCMOTOR_OFFSET_RATE));
    }

    @Override
    public void changeToMove() {
        if (!isActive()) return;
        endRunToPosition();
    }

    @Override
    public void changeToOffset() {
        if (!isActive()) return;
        startRunToPosition();
    }

    @Override
    public void zero() {
        if (!isActive()) return;
        DcMotor.RunMode runMode = getHardwareDevice().getMode();
        getHardwareDevice().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        getHardwareDevice().setTargetPosition(0);
        getHardwareDevice().setMode(runMode);
    }

    @Override
    protected void startFreeze() {
        if (!isActive()) return;
        startRunToPosition();
    }

    @Override
    protected void endFreeze() {
        if (!isActive()) return;
        endRunToPosition();
    }

    @Override
    protected Object getState() {
        if (!isActive()) return null;
        return getHardwareDevice().getCurrentPosition();
    }

    @Override
    protected void applyState(Object state) {
        if (!isActive()) return;
        startRunToPosition();
        setRunToPosition((int) state);
    }

    @Override
    public void addStatusToTelemetry(Telemetry telemetry) {
        if (!isActive()) return;
        telemetry.addData("Device", getHardwareName());
        telemetry.addData("Type", "DC Motor");
        telemetry.addData("Motor power", getHardwareDevice().getPower());
        telemetry.addData("Motor position", getHardwareDevice().getCurrentPosition());
        telemetry.addData("Scaled motor position (rotation)", Calculations.encoderToScaleArmRotation(getHardwareDevice().getCurrentPosition()));
        telemetry.addData("Scaled motor position (raise)", Calculations.encoderToScaleArmRaise(getHardwareDevice().getCurrentPosition()));
        telemetry.addData("Scaled motor position (extension)", Calculations.encoderToScaleArmExtension(getHardwareDevice().getCurrentPosition()));
        telemetry.addLine();
    }

    // Non-overrides

    private boolean isInRunToPositionMode;

    /**
     * Switch the motor to RUN_TO_POSITION mode with the target position as its current position.
     * Does nothing if the motor is already in RUN_TO_POSITION mode.
     */
    private void startRunToPosition() {
        if (isInRunToPositionMode) return;
        setRunToPosition(getHardwareDevice().getCurrentPosition());
        getHardwareDevice().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        getHardwareDevice().setPower(1.0);
        isInRunToPositionMode = true;
    }

    /**
     * Set the target position while the motor is in RUN_TO_POSITION mode.
     * Has no effect if the motor isn't in RUN_TO_POSITION mode.
     */
    private void setRunToPosition(int targetPosition) {
        getHardwareDevice().setTargetPosition(targetPosition);
    }

    /**
     * Switch the motor to RUN_WITHOUT_ENCODER mode.
     * Does nothing if the motor is already in RUN_WITHOUT_ENCODER mode.
     */
    private void endRunToPosition() {
        if (!isInRunToPositionMode) return;
        getHardwareDevice().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        getHardwareDevice().setPower(0.0);
        isInRunToPositionMode = false;
    }
}
