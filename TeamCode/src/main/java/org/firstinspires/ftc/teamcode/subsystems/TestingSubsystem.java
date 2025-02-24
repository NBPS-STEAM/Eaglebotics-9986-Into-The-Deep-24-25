package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helper.testingdevices.TestingCRServo;
import org.firstinspires.ftc.teamcode.helper.testingdevices.TestingDcMotor;
import org.firstinspires.ftc.teamcode.helper.testingdevices.TestingDevice;
import org.firstinspires.ftc.teamcode.helper.testingdevices.TestingNullDevice;
import org.firstinspires.ftc.teamcode.helper.testingdevices.TestingServo;

import java.lang.reflect.InvocationTargetException;
import java.util.*;

/**
 * This is a class for a "subsystem" which tests motors and servos.
 * It detects all hardware devices and allows the driver to choose between any
 * motor, servo, or continuous rotation servo to manipulate.
 *
 * <p>For more information, see TestingTeleOpMode.</p>
 */
public class TestingSubsystem extends SubsystemBase {

    // Private Instance Variables
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private final Map<String, Supplier<?>> extraItems = new LinkedHashMap<>();

    private List<ColorRangeSensor> colorRangeSensors;

    private List<TestingDevice> allDevices;
    private int motorCount;
    private int servoCount;
    private int crServoCount;

    private int selectionIndex;


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
        // Deactivate all devices, if the list exists (list will be null if this is the first time)
        if (allDevices != null) {
            for (TestingDevice device : allDevices) {
                device.deactivate();
            }
        }

        // Reset selection index
        selectionIndex = 0;

        // Recreate and populate allDevices with motors, servos, and continuous rotation (cr) servos
        allDevices = new ArrayList<>();
        motorCount = addAllDevicesAsTesting(allDevices, DcMotor.class, TestingDcMotor.class);
        servoCount = addAllDevicesAsTesting(allDevices, Servo.class, TestingServo.class);
        crServoCount = addAllDevicesAsTesting(allDevices, CRServo.class, TestingCRServo.class);

        // Get all sensors
        colorRangeSensors = hardwareMap.getAll(ColorRangeSensor.class);
        colorRangeSensors.sort(Comparator.comparing(HardwareDevice::getDeviceName)); // Sort by device name
        for (ColorRangeSensor sensor : colorRangeSensors) sensor.enableLed(false);
    }

    /**
     * Determine whether any motors, servos, or continuous rotation servos are configured on the robot.
     * This is true as of initialization or the last time {@link #reloadDeviceList()} was run.
     */
    public boolean hasNoDevices() {
        return allDevices.isEmpty();
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
        selectionIndex = (selectionIndex + inc) % allDevices.size();
        if (selectionIndex < 0)
        {
            selectionIndex += allDevices.size();
        }
    }

    /**
     * @return The TestingDevice at the currently selected index.
     */
    public TestingDevice getSelection() {
        // Sanity check: make sure there are devices
        if (hasNoDevices()) return new TestingNullDevice("");
        return allDevices.get(selectionIndex);
    }

    /**
     * Activate the currently selected device. This will initialize the hardware.
     */
    public void activateSelected() {
        getSelection().activate(hardwareMap);
    }

    /**
     * Deactivate the currently selected device. This will close the hardware.
     */
    public void deactivateSelected() {
        getSelection().deactivate();
    }

    // Device manipulation

    /**
     * Move the currently selected motor, servo, or cr-servo.
     * For a motor or continuous rotation servo, {@code amount} is the amount of power.
     * For a servo, {@code amount} is the target position.
     */
    public void moveSelected(double amount) {
        getSelection().move(amount);
    }

    /**
     * Offset the currently selected motor, servo, or cr-servo.
     * For a motor or servo, {@code amount} is to offset the target position.
     * For a continuous rotation servo, {@code amount} is the amount of power.
     */
    public void offsetSelected(double amount) {
        getSelection().offset(amount);
    }

    /**
     * Zero the currently selected device (if it can be zeroed).
     */
    public void zeroSelected() {
        getSelection().zero();
    }

    /**
     * Store the current state (position/power) of the currently selected hardware device.
     * @param slot The save slot to store to
     */
    public void storeStateSelected(int slot) {
        getSelection().storeState(slot);
    }

    /**
     * Load the stored state (position/power) of the currently selected hardware device.
     * @param slot The save slot to load from
     */
    public void loadStateSelected(int slot) {
        getSelection().loadState(slot);
    }

    /**
     * Freeze the currently selected device so that it stays in place.
     * Calls to moveSelected() are ignored while the device is frozen.
     */
    public void freezeMovement() {
        getSelection().freeze(true);
    }

    /**
     * Unfreeze the currently selected device so that it will move again.
     * @see #freezeMovement()
     */
    public void unfreezeMovement() {
        getSelection().freeze(false);
    }

    /**
     * Freeze the currently selected device if not frozen, otherwise unfreeze.
     * @see #freezeMovement()
     */
    public void toggleFrozen() {
        getSelection().freeze(!getSelection().isFrozen());
    }

    // Telemetry

    /**
     * Add extra data to be reported to telemetry using the .toString() of the value given by the valueProducer.
     * <p>Newly added extra data will overwrite existing extra data with the same caption.</p>
     */
    public<T> void addExtraData(String caption, Supplier<T> valueProducer) {
        extraItems.put(caption, valueProducer);
    }

    public void reportTelemetry() {
        // Clear old data
        telemetry.clear();

        // Give instructions
        telemetry.addLine("If a device is missing from the list, make sure it is configured using 'Configure Robot' on the Driver Station.");
        telemetry.addLine();
        telemetry.addLine("Select a device with the d-pad, then press A to activate.");
        telemetry.addLine("Deactivate the currently selected device by pressing B.");
        telemetry.addLine("Move the selected device with the left stick up/down.");
        telemetry.addLine("Offset the selected device with the right stick up/down.");
        telemetry.addLine("Freeze the selected device in place by pressing X. Press again to unfreeze.");
        telemetry.addLine("Zero the selected device by pressing Y. Servos can't be zeroed.");
        telemetry.addLine("Save the position of the selected device by pressing left bumper. Restore it by pressing right bumper. Hold select while pressing to use the second slot.");
        telemetry.addLine("Reload the list of devices by pressing Start. This checks the configuration in 'Configure Robot', not physically attached hardware.");
        telemetry.addLine();
        telemetry.addLine();

        // List some data
        telemetry.addLine("!! ROBOT TESTING MODE !!");
        telemetry.addLine();
        if (!extraItems.isEmpty()) {
            for (Map.Entry<String, Supplier<?>> item : extraItems.entrySet()) {
                telemetry.addData(item.getKey(), item.getValue().get());
            }
            telemetry.addLine();
        }
        telemetry.addData("Motors found", motorCount);
        telemetry.addData("Servos found", servoCount);
        telemetry.addData("Continuous rotation servos found", crServoCount);
        if (hasNoDevices()) {
            telemetry.addLine();
            telemetry.addLine("WARNING: There are no motors, servos, or continuous rotation servos configured!");
        }
        telemetry.addLine();
        telemetry.addLine();

        // List all devices (indicate whether selected or activated)
        telemetry.addData("Selected index", selectionIndex);
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

        // List all sensor readings
        if (!colorRangeSensors.isEmpty()) {
            telemetry.addLine("Readings of all configured color-range sensors:");
            for (ColorRangeSensor sensor : colorRangeSensors) {
                telemetry.addLine();
                telemetry.addLine(sensor.getDeviceName());
                telemetry.addData("Distance (mm)", sensor.getDistance(DistanceUnit.MM));
                telemetry.addData("Color", TelemetrySubsystem.colorToString(sensor.getNormalizedColors()));
            }
            telemetry.addLine();
            telemetry.addLine();
        }

        // Display info of currently active devices
        telemetry.addLine("All active devices:");
        telemetry.addLine();
        for (TestingDevice device : allDevices) {
            device.addStatusToTelemetry(telemetry);
        }

        // Send data to the Driver Station
        telemetry.update();
    }


    // Helper Methods

    /**
     * Adds all configured devices in the hardware map matching the given {@link HardwareDevice}
     * type to a list as instances of the given {@link TestingDevice} subclass.
     * <p>The added devices are not initialized until {@link TestingDevice#activate(HardwareMap)} is used.</p>
     * @return The number of devices that were added
     */
    // T extends a cry for help
    private<T extends TestingDevice> int addAllDevicesAsTesting(List<T> target, Class<? extends HardwareDevice> hardwareClass, Class<? extends T> testingClass) {
        SortedSet<String> allNames = hardwareMap.getAllNames(hardwareClass);
        for (String device : allNames) {
            try {
                target.add(testingClass.getConstructor(String.class).newInstance(device));
            } catch (NoSuchMethodException | IllegalAccessException | InstantiationException |
                     InvocationTargetException exception) {
                // Oracle, why do these exceptions need to be checked?
            }
        }
        return allNames.size();
    }

    /**
     * Add, to telemetry, the devices named in allDevices within the range {@code start} to {@code end}
     * with a symbol for each one indicating whether it is selected or active.
     */
    private void reportDeviceNames(int start, int end) {
        for (int i = start; i < end; i++) {
            String status = (i == selectionIndex ? ">" : " ") + (allDevices.get(i).isActive() ? "@" : " ");
            telemetry.addData(status, allDevices.get(i).getHardwareName());
        }
    }
}
