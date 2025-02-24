package org.firstinspires.ftc.teamcode.helper.testingdevices;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestingCRServo extends TestingDevice {
    public TestingCRServo(String hardwareName) {
        super(hardwareName);
    }

    @Override
    public CRServo getHardwareDevice() {
        return (CRServo) hardwareDevice;
    }

    @Override
    public void activate(HardwareMap hardwareMap) {
        hardwareDevice = hardwareMap.get(CRServo.class, hardwareName);
        getHardwareDevice().setDirection(CRServo.Direction.FORWARD);
    }

    @Override
    public void deactivate() {
        if (isActive()) getHardwareDevice().getController().pwmDisable();
        super.deactivate();
    }

    @Override
    protected void moveDirectly(double amount) {
        if (!isActive()) return;
        getHardwareDevice().setPower(amount);
    }

    @Override
    protected void offsetDirectly(double amount) {
        if (!isActive()) return;
        getHardwareDevice().setPower(amount);
    }

    @Override
    protected Object getState() {
        if (!isActive()) return null;
        return getHardwareDevice().getPower();
    }

    @Override
    protected void applyState(Object state) {
        if (!isActive()) return;
        getHardwareDevice().setPower((double) state);
    }

    @Override
    public void addStatusToTelemetry(Telemetry telemetry) {
        if (!isActive()) return;
        telemetry.addData("Device", getHardwareName());
        telemetry.addData("Type", "Continuous Rotation Servo");
        telemetry.addData("Continuous rotation servo power", getHardwareDevice().getPower());
        telemetry.addLine();
    }
}
