package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Calculations;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.helper.ResetZeroState;
import org.firstinspires.ftc.teamcode.helper.ArmPosition;
import org.firstinspires.ftc.teamcode.helper.IntakeState;
import org.firstinspires.ftc.teamcode.helper.Wrapper;
import org.jetbrains.annotations.NotNull;

import java.util.*;
import java.util.function.BiConsumer;

/**
 * This is a class for a "subsystem" that manages all of the parts of the arm. It's in charge of
 * applying set positions for the arm's extension, rotation, wrist, and intake.
 */
public class ArmSubsystem extends SubsystemBase {

    // Default motor power
    private final double rotationPower;
    private final double extensionPower;
    private final double raisePower;

    // Positions
    private final HashMap<String, ArmPosition> namedPositions;
    private final Command smartIntakeCommand;
    private Wrapper<String> smartIntakeStow = new Wrapper<>("stow");
    private String lastSetPosition = "";
    private IntakeState intakeState = IntakeState.NONE;
    private double wristOffset = 0.0;
    private boolean isSetPositionLocked = false;
    private boolean invalidateTargets = false;

    // Hardware variables
    private final HardwareMap hardwareMap;
    private final DcMotor rotationMotor;
    private final DcMotor extensionMotor;
    private final DcMotor raiseMotor;
    private final Servo wristServo;
    private final CRServo[] intakeServos;
    private final ColorRangeSensor[] colorRangeSensors;
    /**
     * BE VERY CAREFUL!! If raiseMotor and retractMotor are activated at the same time, the robot WILL tear itself apart.
     * Don't do that! Please!! You must be as safe as possible if you ever use retractMotor.
     */
    private final DcMotor retractMotor; // The motor that retracts the raise


    // Constructor/initialization method
    public ArmSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, Constants.ARM_ROTATION_POWER, Constants.ARM_EXTENSION_POWER, Constants.ARM_RAISE_POWER);
    }
    public ArmSubsystem(HardwareMap hardwareMap, double rotationPower, double extensionPower, double raisePower) {
        this.hardwareMap = hardwareMap;
        // Add all named arm positions
        namedPositions = new HashMap<>();
        // NORMAL POSITION
        addNamedPosition("compact", new ArmPosition(0.17, 2, 0, 1.03, IntakeState.STOPPED));
        addNamedPosition("stow", new ArmPosition(0.37, 2, 0, 1.4, IntakeState.STOPPED));
        addNamedPosition("intake", new ArmPosition(0.37, 67, 0, 1.39, IntakeState.INTAKE));
        addNamedPosition("intake-down", new ArmPosition(0.3, 67, 0, 1.39, IntakeState.INTAKE));
        addNamedPosition("intake vertical", new ArmPosition(0.37, 67, 0, 1.39, IntakeState.INTAKE)); //TODO
        addNamedPosition("intake vertical-down", new ArmPosition(0.3, 67, 0, 1.39, IntakeState.INTAKE)); //TODO
        addNamedPosition("intake ground", new ArmPosition(0.17, 18, 0, 1.317, IntakeState.INTAKE));
        addNamedPosition("intake ground-far", new ArmPosition(0.2, 67, 0, 1.317, IntakeState.INTAKE)); //TODO
        addNamedPosition("basket high", new ArmPosition(0.9, 67, 160, 1.766));
        addNamedPosition("basket low", new ArmPosition(0.7, 2, 160, 1.766));
        addNamedPosition("specimen high", new ArmPosition(0.67, 67, 0, 1.5));
        addNamedPosition("specimen low", new ArmPosition(0.56, 67, 0, 1.5));

        addNamedPosition("hang stage 1", new ArmPosition(1.5, 2, 160, 1.5, IntakeState.STOPPED));
        addNamedPosition("hang stage 2", new ArmPosition(1.8, 2, 0, 1.5, IntakeState.STOPPED));

        // These positions are only used in autonomous routines
        addNamedPosition("ascent level 1", new ArmPosition(0.65, 2, 80, 1.5, IntakeState.STOPPED));
        addNamedPosition("intake ground-high", new ArmPosition(0.17, 67, 95, 1.294, IntakeState.INTAKE)
                .after(() -> applyRotationPositionUnscaled(-50)));

        // ???
        addNamedPosition("pizza", new ArmPosition(Double.NaN, 69, 420, 1.80, IntakeState.OUTTAKE));

        // Get all the hardware using the names set in the Constants file.
        this.rotationMotor = hardwareMap.get(DcMotor.class, Constants.NAME_ARM_ROTATE);
        this.extensionMotor = hardwareMap.get(DcMotor.class, Constants.NAME_ARM_EXTEND);
        this.raiseMotor = hardwareMap.get(DcMotor.class, Constants.NAME_ARM_RAISE);
        this.wristServo = hardwareMap.get(Servo.class, Constants.NAME_ARM_WRIST);
        this.retractMotor = hardwareMap.get(DcMotor.class, Constants.NAME_ARM_RETRACT);

        this.intakeServos = initializeAll(Constants.NAMES_INTAKE, CRServo.class,
                (servo, i) -> servo.setDirection(Constants.DIRECTIONS_INTAKE[i])).toArray(new CRServo[0]);

        this.colorRangeSensors = initializeAll(Constants.NAMES_ARM_COLOR_RANGE, ColorRangeSensor.class,
                (sensor, i) -> sensor.enableLed(false)).toArray(new ColorRangeSensor[0]);


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

        this.retractMotor.setDirection(Constants.DIRECTION_ARM_RETRACT);
        this.retractMotor.setZeroPowerBehavior(Constants.ZEROPOWER_ARM_RETRACT);

        // Zero unless told not to
        if (ResetZeroState.shouldZeroArm()) {
            this.rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.raiseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.retractMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        changeControlModeToRunToPosition(this.rotationMotor, rotationPower);
        changeControlModeToRunToPosition(this.extensionMotor, extensionPower);
        changeControlModeToRunToPosition(this.raiseMotor, raisePower);
        changeControlModeToRunToPosition(this.retractMotor, Constants.ARM_RETRACT_POWER);

        // Prepare smart intake cycle command
        smartIntakeCommand = composeSmartIntakeCommand();
    }


    // Methods

    // Initialization

    /**
     * Try to get and initialize every existing hardware device from an array of names.
     * @return A list of non-null hardware devices that were retrieved
     */
    private<T> List<T> initializeAll(String[] names, Class<T> clazz, BiConsumer<T, Integer> initializer) {
        List<T> list = new ArrayList<>();
        for (int i = 0; i < names.length; i++) {
            T device = hardwareMap.tryGet(clazz, names[i]);
            if (device != null) {
                list.add(device);
                initializer.accept(device, i);
            }
        }
        return list;
    }


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
        if (position.getRaisePosition() > 0) moveRetractToBottom();
        if (position.getAfter() != null) position.getAfter().run();
        invalidateTargets = false;
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

    /**
     * Moves the raise motor to target position 0 or to its current position if it's already below 0.
     */
    public void moveRaiseToBottom() {
        applyRaisePositionUnscaled(Math.min(0, raiseMotor.getCurrentPosition()));
    }

    public void applyWristPosition(double scaled) {
        // The angle for the wrist to point at, on a scale where 1 is up
        applyWristPositionUnscaled(Calculations.scaleToEncoderArmWrist(scaled));
    }

    public void applyWristPositionUnscaled(double encoder) {
        wristServo.setPosition(encoder + wristOffset);
    }

    public void applyIntakeState(IntakeState state) {
        // The state for the intake to go to (stopped, intaking, outtaking)
        // If the state is NONE, then nothing happens. The intake will continue doing what it was doing before.
        switch (state) {
            case STOPPED:
                intakeState = state;
                applyIntakePower(0.0);
                break;
            case INTAKE:
                intakeState = state;
                applyIntakePower(Constants.INTAKE_POWER);
                break;
            case OUTTAKE:
                intakeState = state;
                applyIntakePower(Constants.OUTTAKE_POWER);
                break;
        }
    }

    public void applyIntakePower(double power) {
        for (CRServo servo : intakeServos) {
            servo.setPower(power);
        }
    }

    public void applyRetractPositionUnscaled(int encoder) {
        checkControlModeRunToPosition(retractMotor, Constants.ARM_RETRACT_POWER);
        retractMotor.setTargetPosition(encoder);
    }

    /**
     * Moves the retraction motor to target position 0 or to its current position if it's already below 0.
     */
    public void moveRetractToBottom() {
        applyRetractPositionUnscaled(Math.min(0, retractMotor.getCurrentPosition()));
    }

    public Command cycleHangCommand() {
        return new ConditionalCommand(
                new InstantCommand(() -> applyNamedPosition("hang stage 2")),
                new InstantCommand(() -> applyNamedPosition("hang stage 1")),
                () -> "hang stage 1".equals(getLastSetPosition()));
    }

    public Command intakeDownCommand() {
        return new ConditionalCommand(
                new InstantCommand(() -> applyNamedPosition("intake vertical-down")),
                new InstantCommand(() -> applyNamedPosition("intake-down")),
                () -> "intake vertical".equals(getLastSetPosition()));
    }

    // Controlling the Intake

    public void startIntake() {
        applyIntakeState(IntakeState.INTAKE);
    }

    public void startOuttake() {
        applyIntakeState(IntakeState.OUTTAKE);
    }

    public void stopIntake() {
        applyIntakeState(IntakeState.STOPPED);
    }

    /**
     * @see #hasSampleInIntake()
     */
    public boolean shouldStopIntakeForSample() {
        // Because if the intake is not intaking, it shouldn't stop.
        return intakeState == IntakeState.INTAKE && hasSampleInIntake();
    }
    /**
     * @see #shouldStopIntakeForSample()
     */
    public void intakeIfHasSample() {
        if (shouldStopIntakeForSample()) {
            cycleIntakeSmart();
        }
    }

    /**
     * Cycle through possible actions of the intake depending on the state of the intake and robot.
     * This will run as an uninterruptible command requiring this ArmSubsystem.
     * <p>STOPPED: Start the intake</p>
     * <p>INTAKE: Stop the intake, move to the stow position, then wait briefly (to block consecutive activations)</p>
     * <p>OUTTAKE: Stop the intake, move to the stow position, then wait briefly (to block consecutive activations)</p>
     * <p>The stow position can be customized.</p>
     * @see #setSmartIntakeStowPosition(String)
     */
    public void cycleIntakeSmart() {
        smartIntakeCommand.schedule(false);
    }

    /**
     * Set the position used by the smart intake command to stow. This can be changed at any time.
     * @see #cycleIntakeSmart()
     */
    public void setSmartIntakeStowPosition(String newPositionName) {
        smartIntakeStow.set(newPositionName);
    }

    private Command composeSmartIntakeCommand() {
        // wowza
        ArmSubsystem subsystem = this;
        CommandBase finalCommand = new SelectCommand(
                new HashMap<Object, Command>() {{
                    put(IntakeState.STOPPED, new SequentialCommandGroup(
                            new InstantCommand(subsystem::startIntake)
                    ));
                    put(IntakeState.INTAKE, new SequentialCommandGroup(
                            new InstantCommand(subsystem::stopIntake),
                            new InstantCommand(() -> subsystem.applyNamedPosition(smartIntakeStow.get(), false)),
                            new WaitCommand(500)
                    ));
                    put(IntakeState.OUTTAKE, new SequentialCommandGroup(
                            new InstantCommand(subsystem::stopIntake),
                            new InstantCommand(() -> subsystem.applyNamedPosition(smartIntakeStow.get(), false)),
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

    /** Also sets retract power to 0. */
    public void setRaisePower(double power) {
        setRetractPower(0); // avoid motors pulling against each other
        checkControlModeRunWithoutEncoder(raiseMotor);
        raiseMotor.setPower(power);
    }

    /** Also sets raise power to 0. */
    public void setRetractPower(double power) {
        setRaisePower(0); // avoid motors pulling against each other
        checkControlModeRunWithoutEncoder(retractMotor);
        retractMotor.setPower(power);
    }

    public Command getRunRotationPowerCommand(double power) {
        return new RunMotorPowerCommand(rotationMotor, power);
    }

    public Command getRunExtensionPowerCommand(double power) {
        return new RunMotorPowerCommand(extensionMotor, power);
    }

    public Command getRunRaisePowerCommand(double power) {
        // Lock the retraction motor to go down while the raise motor is active, otherwise they may pull against each other
        return new ParallelCommandGroup(
                new RunMotorPowerCommand(raiseMotor, power),
                new RunCommand(this::moveRetractToBottom)
        );
    }

    public Command getRunRetractPowerCommand(double power) {
        // Lock the raise motor to go down while the retraction motor is active, otherwise they may pull against each other
        return new ParallelCommandGroup(
                new RunMotorPowerCommand(retractMotor, power),
                new RunCommand(this::moveRaiseToBottom)
        );
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

    public void changeWristOffset(double delta) {
        double prepos = getWristPositionUnscaled();
        wristOffset += delta;
        applyWristPositionUnscaled(prepos);
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
        return getRangeReadingMM() < Constants.INTAKE_SAMPLE_THRESHOLD;
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

    /**
     * After running this, all calls to check whether a motor is at its target position will return false
     * until the next set position is applied.
     * <p>This is helpful for autonomous sequences that wait for a set position to be reached and need to avoid
     * ending early before the position is set.</p>
     */
    public void invalidateTargets() {
        invalidateTargets = true;
    }

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

    /** @return Whether the rotation motor is at its target position (default threshold: {@value Constants#ROTATION_TARGET_THRESHOLD} encoder ticks) */
    public boolean isRotationAtTargetPosition() {
        return isRotationAtTargetPosition(Constants.ROTATION_TARGET_THRESHOLD);
    }
    /** @return Whether the rotation motor is at its target position (default threshold: {@value Constants#ROTATION_TARGET_THRESHOLD} encoder ticks) */
    public boolean isRotationAtTargetPosition(int threshold) {
        return isMotorAtTargetPosition(rotationMotor, threshold);
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

    /** @return Whether the extension motor is at its target position (default threshold: {@value Constants#EXTENSION_TARGET_THRESHOLD} encoder ticks) */
    public boolean isExtensionAtTargetPosition() {
        return isExtensionAtTargetPosition(Constants.EXTENSION_TARGET_THRESHOLD);
    }
    /** @return Whether the extension motor is at its target position (default threshold: {@value Constants#EXTENSION_TARGET_THRESHOLD} encoder ticks) */
    public boolean isExtensionAtTargetPosition(int threshold) {
        return isMotorAtTargetPosition(extensionMotor, threshold);
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

    /** @return Whether the raise motor is at its target position (default threshold: {@value Constants#RAISE_TARGET_THRESHOLD} encoder ticks) */
    public boolean isRaiseAtTargetPosition() {
        return isRaiseAtTargetPosition(Constants.RAISE_TARGET_THRESHOLD);
    }
    /** @return Whether the raise motor is at its target position (default threshold: {@value Constants#RAISE_TARGET_THRESHOLD} encoder ticks) */
    public boolean isRaiseAtTargetPosition(int threshold) {
        return isMotorAtTargetPosition(raiseMotor, threshold);
    }


    /** @return The angle that the wrist is pointing at, on a scale where 1 is up */
    public double getWristPosition() {
        return Calculations.encoderToScaleArmWrist(getWristPositionUnscaled());
    }

    /** @return The angle that the wrist is pointing at, unscaled */
    public double getWristPositionUnscaled() {
        return wristServo.getPosition() - wristOffset;
    }

    public double getIntakePower() {
        // The power that the intake servos are moving with (all servos move with the same power)
        if (intakeServos.length == 0) return 0;
        return intakeServos[0].getPower();
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }


    private boolean isMotorAtTargetPosition(DcMotor motor, int threshold) {
        return !invalidateTargets && Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) <= threshold;
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

    public CRServo[] getIntakeServos() {
        return intakeServos;
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
    public Action yieldForRotationTarget(int threshold) {
        return new YieldForMotorTarget(rotationMotor, threshold);
    }

    /**
     * Generates a Road Runner Action that yields until the raise has reached its target position.
     * Used exclusively for Road Runner autonomous routines.
     */
    public Action yieldForRaiseTarget() {
        return yieldForRaiseTarget(Constants.RAISE_TARGET_THRESHOLD);
    }
    public Action yieldForRaiseTarget(int threshold) {
        return new YieldForMotorTarget(raiseMotor, threshold);
    }

    /**
     * Represents a Road Runner Action that yields until a motor has reached its target position.
     * Used exclusively for Road Runner autonomous routines.
     * <p>See "Internal Classes.txt" for more info on internal classes.</p>
     */
    class YieldForMotorTarget implements Action {
        private final DcMotor motor;
        private final int threshold;

        public YieldForMotorTarget(DcMotor motor, int threshold) {
            this.motor = motor;
            this.threshold = threshold;
        }

        @Override
        public boolean run(@NotNull TelemetryPacket packet) {
            return !isMotorAtTargetPosition(motor, threshold); // If true, this action will run again
        }
    }
}
