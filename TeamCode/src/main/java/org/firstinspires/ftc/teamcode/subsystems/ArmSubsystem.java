package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

    // Positions
    private final HashMap<String, ArmPosition> namedPositions;
    private String lastSetPosition = "";

    // Hardware variables
    private final Motor rotationMotor;
    private final Motor extensionMotor;
    private final Motor raiseMotor;
    private final Servo wristServo;
    private final CRServo intakeServo;


    // Constructor/initialization method
    public ArmSubsystem(HardwareMap hardwareMap) {
        // Add all named arm positions
        namedPositions = new HashMap<>();
        addNamedPosition("stow", new ArmPosition(0.15, 0, 0, 0.49, IntakeState.STOP));
        addNamedPosition("intake", new ArmPosition(0.4, 0.4, 0.4, 0.51, IntakeState.INTAKE));
        addNamedPosition("basket low", new ArmPosition(0.4, 0.4, 0.4, 0.4));
        addNamedPosition("basket high", new ArmPosition(0.4, 0.4, 0.4, 0.4));
        addNamedPosition("hang part 1", new ArmPosition(0.4, 0.4, 0.4, 0.4));
        addNamedPosition("hang part 2", new ArmPosition(0.4, 0.4, 0.4, 0.4));

        // Get all of the hardware using the names set in the Constants file.
        this.rotationMotor = new Motor(hardwareMap, Constants.NAME_ARM_ROTATE);
        this.wristServo = hardwareMap.get(Servo.class, Constants.NAME_ARM_WRIST);
        this.intakeServo = hardwareMap.get(CRServo.class, Constants.NAME_INTAKE);
        this.extensionMotor = new Motor(hardwareMap, Constants.NAME_ARM_EXTEND);
        this.raiseMotor = new Motor(hardwareMap, Constants.NAME_ARM_RAISE);


        // Configure hardware
        this.rotationMotor.setInverted(false);
        this.rotationMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.rotationMotor.setRunMode(Motor.RunMode.PositionControl);
        this.rotationMotor.set(Constants.ARM_ROTATION_POWER);
        this.rotationMotor.resetEncoder();

        this.wristServo.setDirection(Servo.Direction.REVERSE);

        this.intakeServo.setDirection(CRServo.Direction.FORWARD);

        this.extensionMotor.setInverted(false);
        this.extensionMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.extensionMotor.setRunMode(Motor.RunMode.PositionControl);
        this.extensionMotor.set(Constants.ARM_EXTENSION_POWER);
        this.extensionMotor.resetEncoder();

        this.raiseMotor.setInverted(false);
        this.raiseMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.raiseMotor.setRunMode(Motor.RunMode.PositionControl);
        this.raiseMotor.set(Constants.ARM_RAISE_POWER);
        this.raiseMotor.resetEncoder();
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
        applyNamedPosition(name, true, true, true, true, true);
    }

    public void applyNamedPosition(String name, boolean doRotation, boolean doExtension, boolean doRaise, boolean doWrist, boolean doIntake) {
        lastSetPosition = name;
        applyPosition(getNamedPosition(name), doRotation, doExtension, doRaise, doWrist, doIntake);
    }

    public void applyPosition(ArmPosition position) {
        applyPosition(position, true, true, true, true, true);
    }

    public void applyPosition(ArmPosition position, boolean doRotation, boolean doExtension, boolean doRaise, boolean doWrist, boolean doIntake) {
        if (doRotation) applyRotationPosition(position.getRotationAngle());
        if (doExtension) applyExtensionPosition(position.getExtensionPosition());
        if (doRaise) applyRaisePosition(position.getRaisePosition());
        if (doWrist) applyWristPosition(position.getWristAngle());
        if (doIntake) applyIntakeState(position.getIntakeState());
    }

    public void applyRotationPosition(double scaled) {
        // The angle for the rotation to point at, on a scale of 0 (straight down) to 1 (straight up)
        // Avoid going below the motor's zero position
        // Remember that the rotation motor's zero position will likely be above 0 on this scale
        rotationMotor.setTargetPosition(Math.max(0, Calculations.scaleToEncoderArmRotation(scaled)));
    }

    public void applyExtensionPosition(double scaled) {
        // The position for the extension to extend to, on a scale of inches
        // Avoid going below the motor's zero position
        extensionMotor.setTargetPosition(Math.max(0, Calculations.scaleToEncoderArmExtension(scaled)));
    }

    public void applyRaisePosition(double scaled) {
        // The position for the extension to extend to, on a scale of inches
        // Avoid going below the motor's zero position
        raiseMotor.setTargetPosition(Math.max(0, Calculations.scaleToEncoderArmRaise(scaled)));
    }

    public void applyWristPosition(double scaled) {
        // The angle for the wrist to point at, on a scale where 1 is up
        wristServo.setPosition(Calculations.scaleToEncoderArmWrist(scaled));
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
        rotationMotor.resetEncoder();
    }

    public void zeroExtensionMotor() {
        extensionMotor.resetEncoder();
    }

    public void zeroRaiseMotor() {
        raiseMotor.resetEncoder();
    }


    // Moving Motors
    // This is a way to move motors without changing them off of PositionControl mode.
    // It doesn't control very nicely, but it's better than nothing.

    public void setRotationTargetDistance(double distance) {
        rotationMotor.setTargetDistance(distance);
    }

    public void setExtensionTargetDistance(double distance) {
        extensionMotor.setTargetDistance(distance);
    }

    public void setRaiseTargetDistance(double distance) {
        raiseMotor.setTargetDistance(distance);
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
        return Calculations.encoderToScaleArmRotation(rotationMotor.getCurrentPosition());
    }

    /** @return The angle that the rotation is pointing at, in encoder ticks */
    public double getRotationPositionUnscaled() {
        return rotationMotor.getCurrentPosition();
    }

    /** @return The position that the extension has extended to, on a scale of inches */
    public double getExtensionPosition() {
        return Calculations.encoderToScaleArmExtension(extensionMotor.getCurrentPosition());
    }

    /** @return The position that the extension has extended to, in encoder ticks */
    public double getExtensionPositionUnscaled() {
        return extensionMotor.getCurrentPosition();
    }

    /** @return The position that the raise has extended to, on a scale of inches */
    public double getRaisePosition() {
        return Calculations.encoderToScaleArmRaise(raiseMotor.getCurrentPosition());
    }

    /** @return The position that the raise has extended to, in encoder ticks */
    public double getRaisePositionUnscaled() {
        return raiseMotor.getCurrentPosition();
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

    public Motor getRotationMotor() {
        return rotationMotor;
    }

    public Motor getExtensionMotor() {
        return extensionMotor;
    }

    public Motor getRaiseMotor() {
        return raiseMotor;
    }

    public Servo getWristServo() {
        return wristServo;
    }

    public CRServo getIntakeServo() {
        return intakeServo;
    }
}
