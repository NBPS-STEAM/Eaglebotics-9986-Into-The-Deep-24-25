package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.helper.ArmPosition;
import org.firstinspires.ftc.teamcode.Calculations;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * This is a class for a "subsystem" that manages all of the parts of the arm. It's in charge of
 * applying set positions, keeping track of the arm's state, and cycling through set positions
 * depending on what state it's in.
 *
 * <p>This subsystem can optionally make use of a finite state machine to cycle through set positions
 * in different states which correspond to different ways to use the arm.
 * For more info, see cycleSetPositionInState() or MecanumTeleOpModeAdvanced.</p>
 *
 * <p>Note: Continuous rotation servos don't support set positions. If one is used, then the
 * extension will not move automatically.</p>
 */
public class ArmSubsystemAdvanced extends SubsystemBase {

    // Enums to label the different states that the arm can be in and their corresponding positions
    public enum ArmState {
        STOW {
            final ArmPosition[] positions = {
                    new ArmPosition(0.18, 0, 1),
            };

            @Override
            public ArmPosition[] getPositions() {return positions;}
        },

        INTAKE {
            final ArmPosition[] positions = {
                    new ArmPosition(0.18, 0, 0.72),
                    new ArmPosition(0.3, 0, 0.72),
            };

            @Override
            public ArmPosition[] getPositions() {return positions;}
        },

        SCORE {
            final ArmPosition[] positions = {
                    new ArmPosition(0.4, 0, 0.90),
                    new ArmPosition(0.5, 0, 0.85),
                    new ArmPosition(0.55, 0, 0.80),
                    new ArmPosition(0.6, 0, 0.76),
                    new ArmPosition(0.65, 0, 0.74),
                    new ArmPosition(0.7, 0, 0.72),
                    new ArmPosition(0.8, 0, 0.68),
            };

            @Override
            public ArmPosition[] getPositions() {return positions;}
        },

        MANUAL,

        UNKNOWN;

        public ArmPosition[] getPositions() {return null;}
    }


    // Hardware variables
    private final DcMotor rotationMotor;
    private final Motor extensionMotor;
    private final CRServo extensionServo;
    private final Servo wristServo;
    private final Servo clawServo;

    // Working variables
    ArmState currentState = ArmState.UNKNOWN;
    int currentPositionInState = 0;


    // Constructor/initialization method
    public ArmSubsystemAdvanced(HardwareMap hardwareMap) {
        this.rotationMotor = hardwareMap.get(DcMotor.class, Constants.NAME_ARM_ROTATE);
        this.wristServo = hardwareMap.get(Servo.class, Constants.NAME_ARM_WRIST);
        this.clawServo = hardwareMap.get(Servo.class, Constants.NAME_CLAW);

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


        this.rotationMotor.setDirection((DcMotor.Direction.FORWARD));
        this.rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        zeroRunToPositionMotor(rotationMotor, Constants.ARM_ROTATION_POWER);

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

        this.wristServo.setDirection(Servo.Direction.REVERSE);

        this.clawServo.setDirection(Servo.Direction.FORWARD);
    }

    // Methods

    /**
     * Cycle through the arm set positions in a particular arm state.
     * <p>If the given state matches the current state, this will increment to that state's next
     * position from the current position.</p>
     * <p>If the given state does not match the current state, this will reset the current position
     * to the first one, then switch to the new state and increment.</p>
     *
     * @param state     The state to cycle through
     * @param increment The number of positions to pass
     */
    public void cycleSetPositionInState(ArmState state, int increment) {
        // ArmSubsystem is fitted with something called a "finite state machine".
        // It's a machine (code) that operates based on a finite number of states.
        // The ArmState enum at the top of this file defines every state the arm can be in and the arm positions available to each one.

        // If in manually controlled state, disable it; the state is changing to something else
        if (isInManualState()) disableManualState();
        // If the new state is different from the current state, change state and reset position to 0
        if (currentState != state) {
            currentState = state;
            currentPositionInState = 0;
        }
        // Increment current position while limiting it to the number of positions in the current state
        // clamp() is defined at the button of this file.
        currentPositionInState = clamp(currentPositionInState + increment, 0, state.getPositions().length - 1);
        // Apply the arm position as the current position of the current state
        applyPosition(state.getPositions()[currentPositionInState], true, true, !isInScoreState());
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
        rotationMotor.setTargetPosition(Math.max(0, Calculations.scaleToEncoderArmRotation(scaled)));
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
        wristServo.setPosition(Calculations.scaleToEncoderArmWrist(scaled));
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

    public void openClaw() {
        clawServo.setPosition(Constants.CLAW_OPEN);
    }

    public void openPartlyClaw() {
        clawServo.setPosition(Constants.CLAW_PARTLY);
    }

    public void closeClaw() {
        clawServo.setPosition(Constants.CLAW_CLOSED);
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

    public void enableManualState() {
        currentState = ArmState.MANUAL;
        currentPositionInState = 0;

        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setPower(0);

        if (hasExtensionMotor()) {
            extensionMotor.set(0);
            extensionMotor.setRunMode(Motor.RunMode.RawPower);
        }
    }

    public void disableManualState() {
        currentState = ArmState.UNKNOWN;
        currentPositionInState = 0;

        rotationMotor.setPower(Constants.ARM_ROTATION_POWER);
        rotationMotor.setTargetPosition(0);
        rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (hasExtensionMotor()) {
            extensionMotor.setTargetPosition(extensionMotor.getCurrentPosition());
            extensionMotor.setRunMode(Motor.RunMode.PositionControl);
            extensionMotor.set(Constants.ARM_EXTENSION_POWER);
        }
    }

    public void toggleManualState() {
        if (isInManualState()) {
            disableManualState();
        } else {
            enableManualState();
        }
    }

    public boolean isInManualState() {
        return currentState == ArmState.MANUAL;
    }

    public boolean isInScoreState() {
        return currentState == ArmState.SCORE;
    }

    public void zeroRotationMotor() {
        if (isInManualState()) {
            zeroRunUsingEncoderMotor(rotationMotor);
        } else {
            zeroRunToPositionMotor(rotationMotor, Constants.ARM_ROTATION_POWER);
        }
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
