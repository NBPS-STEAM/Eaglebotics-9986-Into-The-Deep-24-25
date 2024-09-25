package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Calculations;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.helper.ArmPosition;

/**
 * This is a class for a "subsystem" that manages all of the parts of the arm. It's in charge of
 * applying set positions.
 *
 * <p>This is similar to ArmSubsystemAdvanced, but lacks the finite state machine.</p>
 *
 * <p>Note: Continuous rotation servos don't support set positions. If one is used, then the
 * extension will not move automatically.</p>
 */
public class ArmSubsystemSimple extends SubsystemBase {

    // Positions
    private final ArmPosition stowPosition = new ArmPosition(0.4, 0.4, 0.4);

    // Hardware variables
    private final DcMotor rotationMotor;
    private final Motor extensionMotor;
    private final CRServo extensionServo;
    private final Servo wristServo;
    private final Servo clawServo;


    // Constructor/initialization method
    public ArmSubsystemSimple(HardwareMap hardwareMap) {
        // Usually, you would use hardwareMap.get() to get hardware.
        // That will produce an error if the hardware isn't configured on the robot.
        // For safety, hardwareMap.tryGet() is used instead. This only gives null if it fails.
        this.rotationMotor = hardwareMap.tryGet(DcMotor.class, Constants.NAME_ARM_ROTATE);
        this.wristServo = hardwareMap.tryGet(Servo.class, Constants.NAME_ARM_WRIST);
        this.clawServo = hardwareMap.tryGet(Servo.class, Constants.NAME_CLAW);

        // Extension may be controlled by either a continuous rotation servo or a motor.
        // This will try to get each one, but only if it is in the hardware map.
        // If it isn't in the hardware map, then the variable will be set to null (no data).
        this.extensionServo = hardwareMap.tryGet(CRServo.class, Constants.NAME_ARM_EXTEND_S);
        // Motor doesn't have a method to check if the motor exists, so we must check via hardwareMap.
        DcMotor extMotorCheck = hardwareMap.tryGet(DcMotor.class, Constants.NAME_ARM_EXTEND_M);
        if (extMotorCheck != null) {
            this.extensionMotor = new Motor(hardwareMap, Constants.NAME_ARM_EXTEND_M);
        } else {
            this.extensionMotor = null;
        }

        // Configure hardware

        // Because of hardwareMap.tryGet(), if a piece of hardware couldn't be found, it will be null.
        // Calling a function on null produces an error, so this needs to be checked.
        // If you use hardwareMap.get() instead of hardwareMap.tryGet() (as you should), you can skip the null checks.

        if (this.rotationMotor != null) {
            this.rotationMotor.setDirection((DcMotor.Direction.FORWARD));
            this.rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            zeroRunToPositionMotor(rotationMotor, Constants.ARM_ROTATION_POWER);
        }

        if (this.wristServo != null) {
            this.wristServo.setDirection(Servo.Direction.REVERSE);
        }

        if (this.clawServo != null) {
            this.clawServo.setDirection(Servo.Direction.FORWARD);
        }

        if (hasExtensionMotor()) {
            this.extensionMotor.setInverted(false);
            this.extensionMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            this.extensionMotor.setRunMode(Motor.RunMode.PositionControl);
            this.extensionMotor.set(Constants.ARM_EXTENSION_POWER);
            this.extensionMotor.resetEncoder();
        }

        if (hasExtensionServo()) {
            this.extensionServo.setDirection(CRServo.Direction.REVERSE);
            this.extensionServo.setPower(0);
        }
    }

    // Methods

    public void goToStowPosition() {
        applyPosition(stowPosition);
    }

    public void applyPosition(ArmPosition position) {
        applyPosition(position, true, true, true);
    }

    public void applyPosition(ArmPosition position, boolean doRotation, boolean doExtension, boolean doWrist) {
        if (doRotation) applyRotationPosition(position.getRotationAngle());
        if (doExtension) applyExtensionPosition(position.getExtensionPosition());
        if (doWrist) applyWristPosition(position.getWristAngle());
    }

    public void applyRotationPosition(double scaled) {
        // The angle for the rotation to point at, on a scale of 0 (straight down) to 1 (straight up)
        // Avoid going below the motor's zero position
        // Remember that the rotation motor's zero position will likely be above 0 on this scale
        if (wristServo != null) {
            rotationMotor.setTargetPosition(Math.max(0, Calculations.scaleToEncoderArmRotation(scaled)));
        }
    }

    public void applyExtensionPosition(double scaled) {
        // The position for the extension to extend to, on a scale of inches
        // Avoid going below the motor's zero position
        if (hasExtensionMotor()) {
            extensionMotor.setTargetPosition(Math.max(0, Calculations.scaleToEncoderArmExtension(scaled)));
        }
    }

    public void applyWristPosition(double scaled) {
        // The angle for the wrist to point at, on a scale where 1 is up
        if (wristServo != null) {
            wristServo.setPosition(Calculations.scaleToEncoderArmWrist(scaled));
        }
    }

    public void setRotationMotorPower(double power) {
        rotationMotor.setPower(power);
    }

    public void setExtensionMotorPower(double power) {
        if (hasExtensionMotor()) {
            extensionMotor.set(power);
        }
    }

    public void setExtensionServoPower(double power) {
        if (hasExtensionServo()) {
            extensionServo.setPower(power);
        }
    }

    public void clawSetPositionSafe(double position) {
        if (clawServo != null) {
            clawServo.setPosition(position);
        }
    }

    public void openClaw() {
        clawSetPositionSafe(Constants.CLAW_OPEN);
    }

    public void openPartlyClaw() {
        clawSetPositionSafe(Constants.CLAW_PARTLY);
    }

    public void closeClaw() {
        clawSetPositionSafe(Constants.CLAW_CLOSED);
    }

    public void toggleClaw() {
        /* Servos don't actually tell their position to the Control Hub. getPosition() is always
         equivalent to the last value of setPosition(), so this is an acceptable way to tell if the
         claw is open or closed. This wouldn't work with real encoders since no motor is 100% accurate. */
        if (clawServo.getPosition() == Constants.CLAW_CLOSED) {
            openClaw();
        } else {
            closeClaw();
        }
    }

    public void zeroRotationMotor() {
        zeroRunToPositionMotor(rotationMotor, Constants.ARM_ROTATION_POWER);
    }

    public void zeroExtensionMotor() {
        if (hasExtensionMotor()) {
            extensionMotor.resetEncoder();
        }
    }

    private void zeroRunToPositionMotor(DcMotor motor, double power) {
        // reset, then power, then position, then mode
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setPower(power);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void zeroRunUsingEncoderMotor(DcMotor motor) {
        // reset, then power, then position, then mode
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Methods to check available hardware for arm extension
    public boolean hasExtensionMotor() {
        return extensionMotor != null;
    }

    public boolean hasExtensionServo() {
        return extensionServo != null;
    }

    // Getter methods
    public DcMotor getRotationMotor() {
        return rotationMotor;
    }

    public Motor getExtensionMotor() {
        return extensionMotor;
    }

    public CRServo getExtensionServo() {
        return extensionServo;
    }

    public Servo getWristServo() {
        return wristServo;
    }

    public Servo getClawServo() {
        return clawServo;
    }


    // Helper method to clamp a value within a range, because for some reason this isn't built into Java
    private int clamp(int val, int min, int max) {
        return Math.max(Math.min(val, max), min);
    }
}
