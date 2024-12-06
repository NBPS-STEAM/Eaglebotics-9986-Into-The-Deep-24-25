package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Calculations;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.helper.ArmPosition;
import org.firstinspires.ftc.teamcode.helper.IntakeState;

import java.util.HashMap;

/**
 * This is a class for a "subsystem" that manages all of the parts of the arm. It's in charge of
 * applying set positions for the arm's extension, rotation, wrist, and intake.
 */
public class ArmSubsystem extends SubsystemBase {

    // Static variables
    // Static variables aren't reset between opmodes, only when the robot turns off.
    // This variable is used by the autonomous routine to prevent the robot from resetting after auto.
    public static boolean zeroOnInit = true;

    // Positions
    private final HashMap<String, ArmPosition> namedPositions;
    private String lastSetPosition = "";

    // Hardware variables
    private final DcMotor rotationMotor;
    private final DcMotor extensionMotor;
    private final DcMotor raiseMotor;
    private final Servo wristServo;
    private final CRServo intakeServo;
    private final ColorRangeSensor colorRangeSensor;


    // Constructor/initialization method
    public ArmSubsystem(HardwareMap hardwareMap) {
        // Add all named arm positions
        namedPositions = new HashMap<>();
        addNamedPosition("stow", new ArmPosition(0.3, 2, 0, 0.54, IntakeState.STOP));
        addNamedPosition("intake stage 1", new ArmPosition(0.46, 2, 0, 1));
        addNamedPosition("intake stage 2", new ArmPosition(0.46, 67, 0, 1));
        addNamedPosition("intake stage 3", new ArmPosition(0.36, 67, 0, 1, IntakeState.INTAKE));
        addNamedPosition("basket high", new ArmPosition(0.846, 67, -160, 1));
        addNamedPosition("basket low", new ArmPosition(0.65, 2, -160, 1));
        addNamedPosition("specimen high", new ArmPosition(0.846, 67, -160, 1));
        addNamedPosition("specimen low", new ArmPosition(0.65, 2, -160, 1));

        addNamedPosition("hang stage 1", new ArmPosition(0.9, 2, -160, 0.54, IntakeState.STOP));
        addNamedPosition("hang stage 2", new ArmPosition(0.9, 2, 0, 0.54, IntakeState.STOP));

        addNamedPosition("pizza", new ArmPosition(Double.NaN, 69, 420, 1.80, IntakeState.OUTTAKE));

        // Get all the hardware using the names set in the Constants file.
        this.rotationMotor = hardwareMap.get(DcMotor.class, Constants.NAME_ARM_ROTATE);
        this.extensionMotor = hardwareMap.get(DcMotor.class, Constants.NAME_ARM_EXTEND);
        this.raiseMotor = hardwareMap.get(DcMotor.class, Constants.NAME_ARM_RAISE);
        this.wristServo = hardwareMap.get(Servo.class, Constants.NAME_ARM_WRIST);
        this.intakeServo = hardwareMap.get(CRServo.class, Constants.NAME_INTAKE);
        this.colorRangeSensor = hardwareMap.get(ColorRangeSensor.class, Constants.NAME_ARM_COLOR_RANGE);


        // Configure hardware
        this.rotationMotor.setDirection(DcMotor.Direction.FORWARD);
        this.rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.extensionMotor.setDirection(DcMotor.Direction.FORWARD);
        this.extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.raiseMotor.setDirection(DcMotor.Direction.FORWARD);
        this.raiseMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.wristServo.setDirection(Servo.Direction.FORWARD);

        this.intakeServo.setDirection(CRServo.Direction.FORWARD);

        this.colorRangeSensor.enableLed(false);

        // Zero unless told not to
        if (zeroOnInit) {
            this.rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.raiseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        zeroOnInit = true;

        changeControlModeToRunToPosition(this.rotationMotor, Constants.ARM_ROTATION_POWER);
        changeControlModeToRunToPosition(this.extensionMotor, Constants.ARM_EXTENSION_POWER);
        changeControlModeToRunToPosition(this.raiseMotor, Constants.ARM_RAISE_POWER);
    }


    // Methods

    // Managing Named Arm Positions

    public void addNamedPosition(String name, ArmPosition position) {
        namedPositions.put(name, position);
    }

    public void removeNamedPosition(String name) {
        namedPositions.remove(name);
    }

    public void clearNamedPositions() {
        namedPositions.clear();
    }

    public ArmPosition getNamedPosition(String name) {
        return namedPositions.get(name);
    }

    // Applying Arm Positions

    public void applyNamedPosition(String name) {
        lastSetPosition = name;
        applyPosition(getNamedPosition(name));
    }

    public void applyPosition(ArmPosition position) {
        if (position == null) return; // Skip the rest of this method if an existing position wasn't provided.
        applyRotationPosition(position.getRotationAngle());
        applyExtensionPosition(position.getExtensionPosition());
        applyRaisePosition(position.getRaisePosition());
        applyWristPosition(position.getWristAngle());
        applyIntakeState(position.getIntakeState());
    }

    public void moveMotorsToZero() {
        applyRotationPositionUnscaled(0);
        applyExtensionPositionUnscaled(0);
        applyRaisePositionUnscaled(0);
    }

    public void applyRotationPosition(double scaled) {
        // The angle for the rotation to point at, on a scale of 0 (straight down) to 1 (straight up)
        // Remember that the rotation motor's zero position will likely be above 0 on this scale
        applyRotationPositionUnscaled(Calculations.scaleToEncoderArmRotation(scaled));
    }

    public void applyRotationPositionUnscaled(int encoder) {
        checkControlModeRunToPosition(rotationMotor, Constants.ARM_ROTATION_POWER);
        rotationMotor.setTargetPosition(encoder);
    }

    public void applyExtensionPosition(double scaled) {
        // The position for the extension to extend to, on a scale of inches
        applyExtensionPositionUnscaled(Calculations.scaleToEncoderArmExtension(scaled));
    }

    public void applyExtensionPositionUnscaled(int encoder) {
        checkControlModeRunToPosition(extensionMotor, Constants.ARM_EXTENSION_POWER);
        extensionMotor.setTargetPosition(encoder);
    }

    public void applyRaisePosition(double scaled) {
        // The position for the extension to extend to, on a scale of inches
        applyRaisePositionUnscaled(Calculations.scaleToEncoderArmRaise(scaled));
    }

    public void applyRaisePositionUnscaled(int encoder) {
        checkControlModeRunToPosition(raiseMotor, Constants.ARM_RAISE_POWER);
        raiseMotor.setTargetPosition(encoder);
    }

    public void applyWristPosition(double scaled) {
        // The angle for the wrist to point at, on a scale where 1 is up
        applyWristPositionUnscaled(Calculations.scaleToEncoderArmWrist(scaled));
    }

    public void applyWristPositionUnscaled(double encoder) {
        wristServo.setPosition(encoder);
    }

    public void applyIntakeState(IntakeState state) {
        // The state for the intake to go to (stopped, intaking, outtaking)
        // If the state is NONE, then nothing happens. The intake will continue doing what it was doing before.
        switch (state) {
            case STOP:
                stopIntake();
                break;
            case INTAKE:
                startIntake();
                break;
            case OUTTAKE:
                startOuttake();
                break;
        }
    }

    // Controlling the Intake

    public void startIntake() {
        intakeServo.setPower(Constants.INTAKE_POWER);
    }

    public void startOuttake() {
        intakeServo.setPower(Constants.OUTTAKE_POWER);
    }

    public void stopIntake() {
        intakeServo.setPower(0.0);
    }

    /**
     * @see #hasSampleInIntake()
     */
    public void stopIntakeIfHasSample() {
        if (hasSampleInIntake() && (intakeServo.getPower() * Constants.INTAKE_POWER > 0)) {
            // If it has a sample AND the servo is moving in the INTAKE direction...
            // Because if the servo is outtaking, it shouldn't stop.
            stopIntake();
        }
    }

    public void toggleIntakeIn() {
        toggleIntake(IntakeState.INTAKE);
    }

    public void toggleIntakeOut() {
        toggleIntake(IntakeState.OUTTAKE);
    }

    public void toggleIntake(IntakeState toggleOnState) {
        /* Usually, comparing numbers with decimals using == doesn't work because the slightest
         difference will result in the numbers no longer matching. However, servos don't actually
         measure their position. getPosition() is always equivalent to the last value of
         setPosition(), so this is an adequate way to tell if the intake is moving.
         This wouldn't work with motors because no encoder is 100% accurate. */
        if (intakeServo.getPower() == 0.0) {
            applyIntakeState(toggleOnState);
        } else {
            stopIntake();
        }
    }

    // Zeroing Motors

    public void zeroRotationMotor() {
        this.rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rotationMotor.setTargetPosition(0);
        this.rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rotationMotor.setPower(Constants.ARM_ROTATION_POWER);
    }

    public void zeroExtensionMotor() {
        this.extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.extensionMotor.setTargetPosition(0);
        this.extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.extensionMotor.setPower(Constants.ARM_EXTENSION_POWER);
    }

    public void zeroRaiseMotor() {
        this.raiseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.raiseMotor.setTargetPosition(0);
        this.raiseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.raiseMotor.setPower(Constants.ARM_RAISE_POWER);
    }

    // Moving Motors Without Encoders

    public void setRotationPower(double power) {
        checkControlModeRunWithoutEncoder(rotationMotor);
        rotationMotor.setPower(power);
    }

    public void setExtensionPower(double power) {
        checkControlModeRunWithoutEncoder(extensionMotor);
        extensionMotor.setPower(power);
    }

    public void setRaisePower(double power) {
        checkControlModeRunWithoutEncoder(raiseMotor);
        raiseMotor.setPower(power);
    }


    // Checking and Setting Motor Modes

    private void checkControlModeRunToPosition(DcMotor motor, double power) {
        if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            changeControlModeToRunToPosition(motor, power);
        }
    }

    private void changeControlModeToRunToPosition(DcMotor motor, double power) {
        motor.setTargetPosition(motor.getCurrentPosition());
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    private void checkControlModeRunWithoutEncoder(DcMotor motor) {
        if (motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            changeControlModeToRunWithoutEncoder(motor);
        }
    }

    private void changeControlModeToRunWithoutEncoder(DcMotor motor) {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    // Color/Range Sensor

    /**
     * Gets the color reading of the color/range sensor as a {@link NormalizedRGBA}, which is an
     * object that stores an RGBA color value in a convenient form.
     */
    public NormalizedRGBA getColorReading() {
        return colorRangeSensor.getNormalizedColors();
    }

    /**
     * Determine whether a sample is in the intake, indicated by if something is close to the color/range sensor.
     * @return Whether the distance reading of the color/range sensor is under the threshold set in {@link Constants}
     */
    public boolean hasSampleInIntake() {
        return getRangeReadingMM() < Constants.INTAKE_RANGE_THRESHOLD;
    }

    /**
     * @return The distance reading of the color/range sensor IN MILLIMETERS
     */
    public double getRangeReadingMM() {
        return getRangeReading(DistanceUnit.MM);
    }

    public double getRangeReading(DistanceUnit unit) {
        return colorRangeSensor.getDistance(unit);
    }


    // Getter methods

    // Set Positions

    /**
     * The name of the last set position which was applied by name.
     * This does not track set positions which were not applied by name.
     */
    public String getLastSetPosition() {
        return lastSetPosition;
    }

    // Motor/Servo Positions

    /** @return The angle that the rotation is pointing at, on a scale of 0 (straight down) to 1 (straight up) */
    public double getRotationPosition() {
        return Calculations.encoderToScaleArmRotation(getRotationPositionUnscaled());
    }

    /** @return The angle that the rotation is pointing at, in encoder ticks */
    public int getRotationPositionUnscaled() {
        return rotationMotor.getCurrentPosition();
    }

    /** @return The target angle that the rotation is trying to point at, on a scale of 0 (straight down) to 1 (straight up) */
    public double getRotationTargetPosition() {
        return Calculations.encoderToScaleArmRotation(getRotationTargetPositionUnscaled());
    }

    /** @return The target angle that the rotation is trying to point at, in encoder ticks */
    public int getRotationTargetPositionUnscaled() {
        return rotationMotor.getTargetPosition();
    }


    /** @return The position that the extension has extended to, on a scale of inches */
    public double getExtensionPosition() {
        return Calculations.encoderToScaleArmExtension(getExtensionPositionUnscaled());
    }

    /** @return The position that the extension has extended to, in encoder ticks */
    public int getExtensionPositionUnscaled() {
        return extensionMotor.getCurrentPosition();
    }

    /** @return The target position that the extension is trying to extend to, on a scale of inches */
    public double getExtensionTargetPosition() {
        return Calculations.encoderToScaleArmExtension(getExtensionTargetPositionUnscaled());
    }

    /** @return The target position that the extension is trying to extend to, in encoder ticks */
    public int getExtensionTargetPositionUnscaled() {
        return extensionMotor.getTargetPosition();
    }


    /** @return The position that the raise has extended to, on a scale of inches */
    public double getRaisePosition() {
        return Calculations.encoderToScaleArmRaise(getRaisePositionUnscaled());
    }

    /** @return The position that the raise has extended to, in encoder ticks */
    public int getRaisePositionUnscaled() {
        return raiseMotor.getCurrentPosition();
    }

    /** @return The target position that the raise is trying to extend to, on a scale of inches */
    public double getRaiseTargetPosition() {
        return Calculations.encoderToScaleArmRaise(getRaiseTargetPositionUnscaled());
    }

    /** @return The target position that the raise is trying to extend to, in encoder ticks */
    public int getRaiseTargetPositionUnscaled() {
        return raiseMotor.getTargetPosition();
    }


    /** @return The angle that the wrist is pointing at, on a scale where 1 is up */
    public double getWristPosition() {
        return Calculations.encoderToScaleArmWrist(wristServo.getPosition());
    }

    /** @return The angle that the wrist is pointing at, unscaled */
    public double getWristPositionUnscaled() {
        return wristServo.getPosition();
    }

    public double getIntakePower() {
        // The power that the intake is moving with
        return intakeServo.getPower();
    }

    // Hardware Variables

    public DcMotor getRotationMotor() {
        return rotationMotor;
    }

    public DcMotor getExtensionMotor() {
        return extensionMotor;
    }

    public DcMotor getRaiseMotor() {
        return raiseMotor;
    }

    public Servo getWristServo() {
        return wristServo;
    }

    public CRServo getIntakeServo() {
        return intakeServo;
    }

    public ColorRangeSensor getColorRangeSensor() {
        return colorRangeSensor;
    }
}
