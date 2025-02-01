package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.*;
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
import org.firstinspires.ftc.teamcode.helper.NullColorRangeSensor;
import org.jetbrains.annotations.NotNull;

import java.util.Collections;
import java.util.HashMap;
import java.util.Set;

/**
 * This is a class for a "subsystem" that manages all of the parts of the arm. It's in charge of
 * applying set positions for the arm's extension, rotation, wrist, and intake.
 */
public class ArmSubsystem extends SubsystemBase {

    // Static variables
    // Static variables aren't reset between opmodes, only when the robot turns off.
    // This variable is used by the autonomous routine to prevent the robot from resetting after auto.
    public static boolean zeroOnInit = true;

    // Default motor power
    private final double rotationPower;
    private final double extensionPower;
    private final double raisePower;

    // Positions
    private final HashMap<String, ArmPosition> namedPositions;
    private final Command wibbleWobbleCommand;
    private final Command smartIntakeCommand;
    private String lastSetPosition = "";
    private IntakeState intakeState = IntakeState.NONE;
    private boolean isSetPositionLocked = false;

    // Hardware variables
    private final DcMotor rotationMotor;
    private final DcMotor extensionMotor;
    private final DcMotor raiseMotor;
    private final Servo wristServo;
    private final Servo intakeServo;
    private final ColorRangeSensor[] colorRangeSensors;
    private final DcMotor retractMotor; // The motor that retracts the raise
    // retractMotor was thrown in last-minute and is sloppily implemented. Too bad!


    // Constructor/initialization method
    public ArmSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, Constants.ARM_ROTATION_POWER, Constants.ARM_EXTENSION_POWER, Constants.ARM_RAISE_POWER);
    }
    public ArmSubsystem(HardwareMap hardwareMap, double rotationPower, double extensionPower, double raisePower) {
        // Add all named arm positions
        // TODO: rename wibble wobble 2, remove wibble wobble 1, remove wibble wobble commands
        namedPositions = new HashMap<>();
        addNamedPosition("stow", new ArmPosition(0.4, 2, 0, 1.4));
        addNamedPosition("intake", new ArmPosition(0.4, 67, 0, 1.68, IntakeState.PRIMED_SAMPLE));
        addNamedPosition("the wibble wobble 1", new ArmPosition(0.38, 67, 0, 1.66));
        addNamedPosition("the wibble wobble 2", new ArmPosition(0.36, 67, 0, 1.64, IntakeState.PRIMED_SAMPLE));
        addNamedPosition("intake ground", new ArmPosition(0.17, 21, 0, 1.35, IntakeState.PRIMED_SPECIMEN));
        addNamedPosition("basket high", new ArmPosition(0.9, 68, 160, 1.625));
        addNamedPosition("basket low", new ArmPosition(0.7, 2, 160, 1.55));
        addNamedPosition("specimen high", new ArmPosition(0.7, 2, 0, 1.55));
        addNamedPosition("specimen low", new ArmPosition(0.65, 2, 0, 1.55));

        addNamedPosition("hang stage 1", new ArmPosition(0.5, 2, 160, 1.55));
        addNamedPosition("hang stage 2", new ArmPosition(0.35, 2, 0, 1.55)
                .after(() -> applyRetractPositionUnscaled(10000)));

        // These positions are only used in autonomous routines
        addNamedPosition("compact", new ArmPosition(0.2, 0, 0, 0.54, IntakeState.INTAKE));
        addNamedPosition("intake ground-high", new ArmPosition(0.17, 68, 160, 1.442, IntakeState.OUTTAKE)
                .after(() -> applyRotationPositionUnscaled(-50)));

        // ???
        addNamedPosition("pizza", new ArmPosition(Double.NaN, 69, 420, 1.80, IntakeState.OUTTAKE));

        // Get all the hardware using the names set in the Constants file.
        this.rotationMotor = hardwareMap.get(DcMotor.class, Constants.NAME_ARM_ROTATE);
        this.extensionMotor = hardwareMap.get(DcMotor.class, Constants.NAME_ARM_EXTEND);
        this.raiseMotor = hardwareMap.get(DcMotor.class, Constants.NAME_ARM_RAISE);
        this.wristServo = hardwareMap.get(Servo.class, Constants.NAME_ARM_WRIST);
        this.intakeServo = hardwareMap.get(Servo.class, Constants.NAME_INTAKE);
        this.retractMotor = hardwareMap.get(DcMotor.class, Constants.NAME_ARM_RETRACT);

        this.colorRangeSensors = new ColorRangeSensor[Constants.NAMES_ARM_COLOR_RANGE.length];
        for (int i = 0; i < this.colorRangeSensors.length; i++) {
            this.colorRangeSensors[i] = hardwareMap.tryGet(ColorRangeSensor.class, Constants.NAMES_ARM_COLOR_RANGE[i]);
            if (this.colorRangeSensors[i] == null) this.colorRangeSensors[i] = new NullColorRangeSensor();
            // initialize color range sensors here:
            this.colorRangeSensors[i].enableLed(false);
        }


        // Configure hardware
        this.rotationPower = rotationPower;
        this.extensionPower = extensionPower;
        this.raisePower = raisePower;

        this.rotationMotor.setDirection(Constants.DIRECTION_ARM_ROTATE);
        this.rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.extensionMotor.setDirection(Constants.DIRECTION_ARM_EXTEND);
        this.extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.raiseMotor.setDirection(Constants.DIRECTION_ARM_RAISE);
        this.raiseMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.wristServo.setDirection(Constants.DIRECTION_ARM_WRIST);

        this.intakeServo.setDirection(Constants.DIRECTION_INTAKE);

        this.retractMotor.setDirection(Constants.DIRECTION_ARM_RETRACT);
        this.retractMotor.setZeroPowerBehavior(Constants.ZEROPOWER_ARM_RETRACT);

        // Zero unless told not to
        if (zeroOnInit) {
            this.rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.raiseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.retractMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        zeroOnInit = true;

        changeControlModeToRunToPosition(this.rotationMotor, rotationPower);
        changeControlModeToRunToPosition(this.extensionMotor, extensionPower);
        changeControlModeToRunToPosition(this.raiseMotor, raisePower);
        changeControlModeToRunToPosition(this.retractMotor, Constants.ARM_RETRACT_POWER);

        // Prepare smart intake cycle command
        wibbleWobbleCommand = composeWibbleWobbleCommand();
        smartIntakeCommand = composeSmartIntakeCommand();
    }


    // Methods

    // Managing Named Arm Positions

    public void lockSetPosition(boolean locked) {
        isSetPositionLocked = locked;
    }

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
        applyNamedPosition(name, true);
    }

    public void applyNamedPosition(String name, boolean interruptCommand) {
        applyNamedPosition(name, interruptCommand, false);
    }

    /**
     * Runs regardless of whether a command is scheduled, but if interruptCommand is true, then this will
     * attempt to interrupt the current command by scheduling an empty instant command.
     */
    public void applyNamedPosition(String name, boolean interruptCommand, boolean lockSetPosition) {
        lastSetPosition = name;
        applyPosition(getNamedPosition(name), interruptCommand, lockSetPosition);
    }

    /**
     * Runs regardless of whether a command is scheduled, but if interruptCommand is true, then this will
     * attempt to interrupt the current command by scheduling an empty instant command.
     */
    public void applyPosition(ArmPosition position, boolean interruptCommand, boolean lockSetPosition) {
        if (isSetPositionLocked) return;
        lockSetPosition(lockSetPosition);
        //if (interruptCommand) new InstantCommand(() -> {}, this).schedule(true);
        if (position == null) return; // Skip the rest of this method if an existing position wasn't provided.
        applyRotationPosition(position.getRotationAngle());
        applyExtensionPosition(position.getExtensionPosition());
        applyRaisePosition(position.getRaisePosition());
        applyWristPosition(position.getWristAngle());
        applyIntakeState(position.getIntakeState());
        applyRetractPositionUnscaled(0);
        if (position.getAfter() != null) position.getAfter().run();
    }

    public void moveMotorsToZero() {
        applyRotationPositionUnscaled(0);
        applyExtensionPositionUnscaled(0);
        applyRaisePositionUnscaled(0);
        applyRetractPositionUnscaled(0);
    }

    public void applyRotationPosition(double scaled) {
        // The angle for the rotation to point at, on a scale of 0 (straight down) to 1 (straight up)
        // Remember that the rotation motor's zero position will likely be above 0 on this scale
        applyRotationPositionUnscaled(Calculations.scaleToEncoderArmRotation(scaled));
    }

    public void applyRotationPositionUnscaled(int encoder) {
        checkControlModeRunToPosition(rotationMotor, rotationPower);
        rotationMotor.setTargetPosition(encoder);
    }

    public void applyExtensionPosition(double scaled) {
        // The position for the extension to extend to, on a scale of inches
        applyExtensionPositionUnscaled(Calculations.scaleToEncoderArmExtension(scaled));
    }

    public void applyExtensionPositionUnscaled(int encoder) {
        checkControlModeRunToPosition(extensionMotor, extensionPower);
        extensionMotor.setTargetPosition(encoder);
    }

    public void applyRaisePosition(double scaled) {
        // The position for the extension to extend to, on a scale of inches
        applyRaisePositionUnscaled(Calculations.scaleToEncoderArmRaise(scaled));
    }

    public void applyRaisePositionUnscaled(int encoder) {
        checkControlModeRunToPosition(raiseMotor, raisePower);
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
        // The state for the intake to go to (primed, intaking, outtaking)
        // If the state is NONE, then nothing happens. The intake will continue doing what it was doing before.
        switch (state) {
            case PRIMED_SAMPLE:
            case PRIMED_SPECIMEN:
                if (intakeState != IntakeState.INTAKE) {
                    intakeState = state;
                    intakeServo.setPosition(Constants.INTAKE_PRIME_POSITION);
                }
                break;
            case INTAKE:
                intakeState = state;
                intakeServo.setPosition(Constants.INTAKE_POSITION);
                break;
            case OUTTAKE:
                intakeState = state;
                intakeServo.setPosition(Constants.OUTTAKE_POSITION);
                break;
        }
    }

    public void applyRetractPositionUnscaled(int encoder) {
        checkControlModeRunToPosition(retractMotor, Constants.ARM_RETRACT_POWER);
        retractMotor.setTargetPosition(encoder);
    }

    public void cycleHang() {
        if ("hang stage 1".equals(getLastSetPosition())) {
            applyNamedPosition("hang stage 2", true, true);
        } else {
            applyNamedPosition("hang stage 1");
        }
    }

    // Special Actions

    public void doTheWibbleWobble() {
        wibbleWobbleCommand.schedule(true);
    }

    private Command composeWibbleWobbleCommand() {
        CommandBase command = new RepeatCommand(new SequentialCommandGroup(
                new InstantCommand(() -> applyNamedPosition("the wibble wobble 1", false)),
                new WaitCommand(250),
                new InstantCommand(() -> applyNamedPosition("the wibble wobble 2", false)),
                new WaitCommand(250)
        ));
        command.addRequirements(this);
        return command;
    }

    // Controlling the Intake

    public void startIntake() {
        applyIntakeState(IntakeState.INTAKE);
    }

    public void startOuttake() {
        applyIntakeState(IntakeState.OUTTAKE);
    }

    public void primeIntakeSample() {
        applyIntakeState(IntakeState.PRIMED_SAMPLE);
    }

    public void primeIntakeSpecimen() {
        applyIntakeState(IntakeState.PRIMED_SPECIMEN);
    }

    /**
     * @see #hasSampleInIntake()
     */
    public void intakeIfHasSample() {
        if ((intakeState == IntakeState.PRIMED_SAMPLE || intakeState == IntakeState.PRIMED_SPECIMEN) && hasSampleInIntake()) {
            // Because if the servo is not primed, it shouldn't intake.
            cycleIntakeSmart();
        }
    }

    /**
     * Cycle through possible actions of the intake depending on the state of the intake and robot.
     * This will run as an uninterruptible command requiring this ArmSubsystem.
     * <p>OUTTAKE: Prime the intake and watch for samples</p>
     * <p>PRIMED: Close the intake and wait briefly, then move to "stow" position</p>
     * <p>INTAKE: Open the intake and wait briefly (to block consecutive activations)</p>
     */
    public void cycleIntakeSmart() {
        smartIntakeCommand.schedule(false);
    }

    private Command composeSmartIntakeCommand() {
        // wowza
        ArmSubsystem subsystem = this;
        CommandBase primeCommand = new SequentialCommandGroup(
                new InstantCommand(subsystem::startIntake),
                new WaitCommand(500),
                new InstantCommand(() -> subsystem.applyNamedPosition("stow", false))
        );
        CommandBase finalCommand = new SelectCommand(
                new HashMap<Object, Command>() {{
                    put(IntakeState.OUTTAKE, new SequentialCommandGroup(
                            new InstantCommand(subsystem::primeIntakeSample)
                    ));
                    put(IntakeState.PRIMED_SAMPLE, primeCommand);
                    put(IntakeState.PRIMED_SPECIMEN, primeCommand);
                    put(IntakeState.INTAKE, new SequentialCommandGroup(
                            new InstantCommand(subsystem::startOuttake),
                            new WaitCommand(500)
                    ));
                }},
                this::getIntakeState
        );
        finalCommand.addRequirements(this);
        return finalCommand;
    }

    // Zeroing Motors

    public void zeroRotationMotor() {
        this.rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rotationMotor.setTargetPosition(0);
        this.rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rotationMotor.setPower(rotationPower);
    }

    public void zeroExtensionMotor() {
        this.extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.extensionMotor.setTargetPosition(0);
        this.extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.extensionMotor.setPower(extensionPower);
    }

    public void zeroRaiseMotor() {
        this.raiseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.raiseMotor.setTargetPosition(0);
        this.raiseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.raiseMotor.setPower(raisePower);
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

    public Command getRunRotationPowerCommand(double power) {
        return new RunMotorPowerCommand(rotationMotor, power);
    }

    public Command getRunExtensionPowerCommand(double power) {
        return new RunMotorPowerCommand(extensionMotor, power);
    }

    public Command getRunRaisePowerCommand(double power) {
        return new RunMotorPowerCommand(raiseMotor, power);
    }

    public Command getRunRetractPowerCommand(double power) {
        return new RunMotorPowerCommand(retractMotor, power);
    }

    class RunMotorPowerCommand implements Command {
        private final DcMotor motor;
        private final double power;

        public RunMotorPowerCommand(DcMotor motor, double power) {
            this.motor = motor;
            this.power = power;
        }

        @Override
        public void initialize() {
            checkControlModeRunWithoutEncoder(motor);
            motor.setPower(power);
        }

        @Override
        public void end(boolean interrupted) {
            checkControlModeRunWithoutEncoder(motor);
            motor.setPower(0);
        }

        @Override
        public Set<Subsystem> getRequirements() {
            return Collections.emptySet();
        }
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
     * Gets the color reading of each color/range sensor as a {@link NormalizedRGBA}, which is an
     * object that stores an RGBA color value in a convenient form.
     */
    public NormalizedRGBA[] getColorReadings() {
        NormalizedRGBA[] arr = new NormalizedRGBA[colorRangeSensors.length];
        for (int i = 0; i < arr.length; i++) {
            arr[i] = colorRangeSensors[i].getNormalizedColors();
        }
        return arr;
    }

    /**
     * Determine whether a sample is in the intake, indicated by if something is close to the color/range sensor.
     * @return Whether the distance reading of the color/range sensor is under the threshold set in {@link Constants}
     */
    public boolean hasSampleInIntake() {
        if (intakeState == IntakeState.PRIMED_SPECIMEN) {
            return getRangeReadingMM() < Constants.INTAKE_SPECIMEN_THRESHOLD;
        } else {
            return getRangeReadingMM() < Constants.INTAKE_SAMPLE_THRESHOLD;
        }
    }

    /**
     * @return The distance reading of the CLOSEST color/range sensor IN MILLIMETERS
     */
    public double getRangeReadingMM() {
        return getRangeReading(DistanceUnit.MM);
    }

    /**
     * @param unit  The unit of measurement of the reading
     * @see #getRangeReadingMM()
     */
    public double getRangeReading(DistanceUnit unit) {
        double min = DistanceUnit.infinity;
        for (ColorRangeSensor sensor : colorRangeSensors) {
            min = Math.min(min, sensor.getDistance(unit));
        }
        return min;
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

    public double getIntakePosition() {
        // The power that the intake is moving with
        return intakeServo.getPosition();
    }

    public IntakeState getIntakeState() {
        return intakeState;
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

    public Servo getIntakeServo() {
        return intakeServo;
    }

    public ColorRangeSensor[] getColorRangeSensors() {
        return colorRangeSensors;
    }


    // Road Runner Actions
    // For more info:
    // https://rr.brott.dev/docs/v1-0/guides/centerstage-auto/
    // https://rr.brott.dev/docs/v1-0/actions/

    /**
     * Generates a Road Runner Action that yields until the rotation has reached its target position.
     * Used exclusively for Road Runner autonomous routines.
     */
    public Action yieldForRotationTarget() {
        return yieldForRotationTarget(Constants.ROTATION_TARGET_THRESHOLD);
    }
    public Action yieldForRotationTarget(double threshold) {
        return new YieldForMotorTarget(getRotationMotor(), threshold);
    }

    /**
     * Generates a Road Runner Action that yields until the raise has reached its target position.
     * Used exclusively for Road Runner autonomous routines.
     */
    public Action yieldForRaiseTarget() {
        return yieldForRaiseTarget(Constants.RAISE_TARGET_THRESHOLD);
    }
    public Action yieldForRaiseTarget(double threshold) {
        return new YieldForMotorTarget(getRaiseMotor(), threshold);
    }

    /**
     * Represents a Road Runner Action that yields until a motor has reached its target position.
     * Used exclusively for Road Runner autonomous routines.
     * <p>Usually, inner classes (such as YieldForMotorTarget) must be tied to an instance of the outer class but may
     * use variables/methods from that outer class. {@code static class} bypasses this, allowing you to create instances
     * of the inner class without an instance of the outer class.</p>
     * <p>i.e. If this class were not static, then to create a new instance, you may do something like:</p>
     * <p>{@code new ArmSubsystem().new YieldForMotorTarget()}</p>
     * However, because it is static, you can instead just do:
     * <p>{@code new ArmSubsystem.YieldForMotorTarget()}</p>
     * <p>Note: Because new YieldForMotorTargets are only ever created in its outer class (this class), the fact that
     * it's static doesn't actually change anything. But now you know what static classes are!</p>
     */
    static class YieldForMotorTarget implements Action {
        private final DcMotor motor;
        private final double threshold;

        public YieldForMotorTarget(DcMotor motor, double threshold) {
            this.motor = motor;
            this.threshold = threshold;
        }

        @Override
        public boolean run(@NotNull TelemetryPacket packet) {
            return Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) > threshold; // If true, this action will run again
        }
    }
}
