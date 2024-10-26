package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Pair;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Calculations;

import java.util.ArrayList;
import java.util.List;
import java.util.SortedSet;

/**
 * This is a class for a "subsystem" which tests motors and servos.
 * It detects all hardware devices and allows the driver to choose between any
 * motor, servo, or continuous rotation servo to manipulate.
 *
 * <p>For more information, see TestingTeleOpMode.</p>
 */
public class TestingSubsystem extends SubsystemBase {
    enum HardwareOption {
        NONE,
        DC_MOTOR,
        SERVO,
        CR_SERVO,
    }


    // Private Instance Variables
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private List<Pair<String, HardwareOption>> allDevices;
    private int motorCount;
    private int servoCount;
    private int crServoCount;

    private int selectedDeviceIndex;
    private boolean isDeviceFrozen;

    private int currentDeviceIndex;
    private HardwareOption currentHardware;
    private Object currentDevice;


    // Constructor methods
    public TestingSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        reloadDeviceList();
    }


    // Methods

    // Device selection

    /**
     * Reload the list of motors, servos, and continuous rotation servos configured on the robot.
     * This checks using the list of configured devices. Hardware changes (i.e. plugging/unplugging)
     * will not be reflected in the new list unless the configuration is changed too.
     * <p>This will deactivate the currently activated device, if there is one.</p>
     * <p>This is run on initialization.</p>
     */
    public void reloadDeviceList() {
        deactivateCurrent();

        selectedDeviceIndex = 0;

        allDevices = new ArrayList<>();
        motorCount = addAllDeviceNames(DcMotor.class, HardwareOption.DC_MOTOR);
        servoCount = addAllDeviceNames(Servo.class, HardwareOption.SERVO);
        crServoCount = addAllDeviceNames(CRServo.class, HardwareOption.CR_SERVO);
    }

    /**
     * Determine whether any motors, servos, or continuous rotation servos are configured on the robot.
     * This is true as of initialization or the last time {@link #reloadDeviceList()} was run.
     */
    public boolean hasNoDevices() {
        return allDevices.isEmpty();
    }

    /**
     * @return Whether there is an active device
     */
    public boolean isActiveDevice() {
        return currentDevice != null;
    }

    /**
     * Increment the index of the selected device by {@code inc} places.
     * If the new index would be out of bounds, it will wrap around.
     */
    public void changeSelection(int inc) {
        // Sanity check: make sure there are devices
        if (hasNoDevices()) return;
        // % is the modulus operator.
        // If selectedDevice + inc is out of bounds of allDevices, it will wrap around.
        selectedDeviceIndex = (selectedDeviceIndex + inc) % allDevices.size();
    }

    /**
     * Activate the currently selected device. This will initialize the hardware.
     */
    public void activateSelected() {
        // Sanity check: make sure there are devices
        if (hasNoDevices()) return;

        deactivateCurrent();

        currentDeviceIndex = selectedDeviceIndex;
        currentHardware = allDevices.get(selectedDeviceIndex).second;
        currentDevice = null;

        switch (currentHardware) {
            case DC_MOTOR:
                DcMotor motor = hardwareMap.get(DcMotor.class, allDevices.get(selectedDeviceIndex).first);
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setDirection(DcMotor.Direction.FORWARD);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                currentDevice = motor;
                break;

            case SERVO:
                Servo servo = hardwareMap.get(Servo.class, allDevices.get(selectedDeviceIndex).first);
                servo.setDirection(Servo.Direction.FORWARD);
                currentDevice = servo;
                break;

            case CR_SERVO:
                CRServo crServo = hardwareMap.get(CRServo.class, allDevices.get(selectedDeviceIndex).first);
                crServo.setDirection(CRServo.Direction.FORWARD);
                currentDevice = crServo;
                break;
        }
    }

    /**
     * Deactivate the currently activated device. This will close the hardware.
     */
    public void deactivateCurrent() {
        if (currentDevice != null) { // sanity check; probably not necessary
            switch (currentHardware) {
                case DC_MOTOR:
                    ((DcMotor) currentDevice).close();
                    break;
                case SERVO:
                    ((Servo) currentDevice).close();
                    break;
                case CR_SERVO:
                    ((CRServo) currentDevice).close();
                    break;
            }
        }
        currentDeviceIndex = -1;
        currentHardware = HardwareOption.NONE;
        currentDevice = null;
    }

    // Device manipulation

    /**
     * Move the currently activated motor, servo, or cr-servo.
     * For a motor or continuous rotation servo, {@code amount} is the amount of power.
     * For a servo, {@code amount} is the target position.
     */
    public void moveCurrent(double amount) {
        if (isFrozen()) return;
        switch (currentHardware) {
            case DC_MOTOR:
                ((DcMotor) currentDevice).setPower(amount);
                break;
            case SERVO:
                ((Servo) currentDevice).setPosition(amount);
                break;
            case CR_SERVO:
                ((CRServo) currentDevice).setPower(amount);
                break;
        }
    }

    /**
     * Zero the currently activated device (if it can be zeroed).
     */
    public void zeroCurrent() {
        switch (currentHardware) {
            case DC_MOTOR:
                ((DcMotor) currentDevice).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ((DcMotor) currentDevice).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
        }
    }

    /**
     * Freeze the currently activated device at the amount of power/position given here.
     * Calls to moveCurrent() are ignored while the device is frozen.
     * Frozen status is not reset when the active device changes!
     */
    public void freezeMovement() {
        isDeviceFrozen = true;
    }

    /**
     * Unfreeze the currently activated device, so that calls to moveCurrent() obey the {@code amount} given there.
     * @see #freezeMovement()
     */
    public void unfreezeMovement() {
        isDeviceFrozen = false;
    }

    /**
     * Whether the currently activated device is frozen.
     * @see #freezeMovement()
     */
    public boolean isFrozen() {
        return isDeviceFrozen;
    }

    /**
     * Freeze the currently activated device with this amount if not frozen, otherwise unfreeze.
     * @see #freezeMovement()
     */
    public void toggleFrozen() {
        if (isFrozen()) unfreezeMovement();
        else freezeMovement();
    }

    // Telemetry

    public void reportTelemetry() {
        // Clear old data
        telemetry.clear();

        // List some data
        telemetry.addLine("!! ROBOT TESTING MODE !!");
        telemetry.addLine();
        telemetry.addData("Motors found", motorCount);
        telemetry.addData("Servos found", servoCount);
        telemetry.addData("Continuous rotation servos found", crServoCount);
        if (hasNoDevices()) {
            telemetry.addLine();
            telemetry.addLine("WARNING: There are no motors, servos, or continuous rotation servos configured!");
        }
        telemetry.addLine();
        telemetry.addLine();

        // Give instructions
        telemetry.addLine("If a device is missing from the list, make sure it is configured using 'Configure Robot' on the Driver Station.");
        telemetry.addLine();
        telemetry.addLine("Select a device with the d-pad, then press A to activate.");
        telemetry.addLine("Deactivate the currently active device by pressing B.");
        telemetry.addLine("Move the active device with the left stick up/down.");
        telemetry.addLine("Freeze the position of the left stick by pressing X. Press again to unfreeze.");
        telemetry.addLine("Zero the active device by pressing Y. Servos can't be zeroed.");
        telemetry.addLine("Reload the list of devices by pressing Start. This checks the configuration in 'Configure Robot', not physically attached hardware.");
        telemetry.addLine();
        telemetry.addLine();

        // List all devices (indicate whether selected or activated)
        telemetry.addData("Selected index", selectedDeviceIndex);
        telemetry.addLine();
        telemetry.addLine("All configured motors:");
        reportDeviceNames(0, motorCount);
        telemetry.addLine();
        telemetry.addLine("All configured servos:");
        reportDeviceNames(motorCount, motorCount + servoCount);
        telemetry.addLine();
        telemetry.addLine("All configured continuous rotation servos:");
        reportDeviceNames(motorCount + servoCount, motorCount + servoCount + crServoCount);
        telemetry.addLine();
        telemetry.addLine();

        // Display info of currently active device
        if (isActiveDevice()) {
            reportActiveDevice();
        }

        // Send data to the Driver Station
        telemetry.update();
    }


    // Helper Methods

    /**
     * Adds the names of all configured devices in the hardware map matching
     * the given type to allDevices with the given HardwareOption type.
     * @return The number of names that were added
     */
    private int addAllDeviceNames(Class<? extends HardwareDevice> clazz, HardwareOption type) {
        SortedSet<String> allNames = hardwareMap.getAllNames(clazz);
        for (String device : allNames) {
            allDevices.add(new Pair<>(device, type));
        }
        return allNames.size();
    }

    /**
     * Add, to telemetry, the devices named in allDevices within the range {@code start} to {@code end}
     * with a symbol for each one indicating whether it is selected or active.
     */
    private void reportDeviceNames(int start, int end) {
        for (int i = start; i < end; i++) {
            String status = (i == selectedDeviceIndex ? ">" : " ") + (i == currentDeviceIndex ? "@" : " ");
            telemetry.addData(status, allDevices.get(i).first);
        }
    }

    private void reportActiveDevice() {
        telemetry.addData("Currently active device", allDevices.get(currentDeviceIndex).first + " (" + currentDevice.getClass().getSimpleName() + ")");
        switch (currentHardware) {
            case DC_MOTOR:
                telemetry.addData("Motor power", ((DcMotor) currentDevice).getPower());
                telemetry.addData("Motor position", ((DcMotor) currentDevice).getCurrentPosition());
                telemetry.addData("Scaled motor position (rotation)", Calculations.encoderToScaleArmRotation(((DcMotor) currentDevice).getCurrentPosition()));
                telemetry.addData("Scaled motor position (raise)", Calculations.encoderToScaleArmRaise(((DcMotor) currentDevice).getCurrentPosition()));
                telemetry.addData("Scaled motor position (extension)", Calculations.encoderToScaleArmExtension(((DcMotor) currentDevice).getCurrentPosition()));
                break;
            case SERVO:
                telemetry.addData("Servo position", ((Servo) currentDevice).getPosition());
                telemetry.addData("Scaled servo position (wrist)", Calculations.encoderToScaleArmWrist(((Servo) currentDevice).getPosition()));
                break;
            case CR_SERVO:
                telemetry.addData("Continuous rotation servo power", ((CRServo) currentDevice).getPower());
                break;
        }
    }
}
