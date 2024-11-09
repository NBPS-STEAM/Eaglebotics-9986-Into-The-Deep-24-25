package org.firstinspires.ftc.teamcode.helper.testingdevices;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A special TestingDevice that does not correspond to any real hardware device.
 * All calls to all methods will do nothing.
 */
public class TestingNullDevice extends TestingDevice {
    public TestingNullDevice(String hardwareName) {
        super(hardwareName);
    }

    @Override
    public void activate(HardwareMap hardwareMap) {}

    @Override
    protected void moveDirectly(double amount) {}

    @Override
    protected void offsetDirectly(double amount) {}

    @Override
    protected Object getState() { return null; }

    @Override
    protected void applyState(Object state) {}

    @Override
    public void addStatusToTelemetry(Telemetry telemetry) {}
}
