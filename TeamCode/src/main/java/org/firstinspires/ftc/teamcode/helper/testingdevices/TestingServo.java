package org.firstinspires.ftc.teamcode.helper.testingdevices;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Calculations;
import org.firstinspires.ftc.teamcode.Constants;

public class TestingServo extends TestingDevice {
    public TestingServo(String hardwareName) {
        super(hardwareName);
    }

    @Override
    public Servo getHardwareDevice() {
        return (Servo) hardwareDevice;
    }

    @Override
    public void activate(HardwareMap hardwareMap) {
        hardwareDevice = hardwareMap.get(Servo.class, hardwareName);
        getHardwareDevice().setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void deactivate() {
        if (isActive()) getHardwareDevice().getController().pwmDisable();
        super.deactivate();
    }

    @Override
    protected void moveDirectly(double amount) {
        if (!isActive()) return;
        getHardwareDevice().setPosition(amount);
    }

    @Override
    protected void offsetDirectly(double amount) {
        if (!isActive()) return;
        getHardwareDevice().setPosition(getHardwareDevice().getPosition() + amount * Constants.TESTING_SERVO_OFFSET_RATE);
    }

    @Override
    protected Object getState() {
        if (!isActive()) return null;
        return getHardwareDevice().getPosition();
    }

    @Override
    protected void applyState(Object state) {
        if (!isActive()) return;
        getHardwareDevice().setPosition((double) state);
    }

    @Override
    public void addStatusToTelemetry(Telemetry telemetry) {
        if (!isActive()) return;
        telemetry.addData("Device", getHardwareName());
        telemetry.addData("Type", "Servo");
        telemetry.addData("Servo position", getHardwareDevice().getPosition());
        telemetry.addData("Scaled servo position (wrist)", Calculations.encoderToScaleArmWrist(getHardwareDevice().getPosition()));
        telemetry.addLine();
    }
}
