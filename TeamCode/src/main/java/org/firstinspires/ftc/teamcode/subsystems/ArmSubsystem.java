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
 * <p>The arm motor still has to be set to a port on the Control/Expansion Hub even if a servo is
 * being used instead because im lazy and don't feel like disabling the motor code</p>
 *
 * <p>Note: Continuous rotation servos don't support set positions. If one is used, then the
 * extension will not move automatically.</p>
 */
public class ArmSubsystem extends SubsystemBase {

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
    private final Servo leftClawServo;
    private final Servo rightClawServo;

    // Working variables
    ArmState currentState = ArmState.UNKNOWN;
    int currentPositionInState = 0;


    // Constructor method
    public ArmSubsystem(HardwareMap hardwareMap) {
        this.rotationMotor = hardwareMap.get(DcMotor.class, Constants.NAME_ARM_ROTATE);
        this.extensionMotor = new Motor(hardwareMap, Constants.NAME_ARM_EXTEND_M);
        this.extensionServo = hardwareMap.get(CRServo.class, Constants.NAME_ARM_EXTEND_S);
        this.wristServo = hardwareMap.get(Servo.class, Constants.NAME_ARM_WRIST);
        this.leftClawServo = hardwareMap.get(Servo.class, Constants.NAME_CLAW_L);
        this.rightClawServo = hardwareMap.get(Servo.class, Constants.NAME_CLAW_R);


        this.rotationMotor.setDirection((DcMotor.Direction.FORWARD));
        this.rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        zeroRunToPositionMotor(rotationMotor, Constants.ARM_ROTATION_POWER);

        this.extensionMotor.setInverted(false);
        this.extensionMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.extensionMotor.setRunMode(Motor.RunMode.PositionControl);
        this.extensionMotor.set(Constants.ARM_EXTENSION_POWER);
        this.extensionMotor.resetEncoder();

        this.extensionServo.setDirection(CRServo.Direction.REVERSE);
        this.extensionServo.setPower(0);

        this.wristServo.setDirection(Servo.Direction.REVERSE);

        this.leftClawServo.setDirection(Servo.Direction.REVERSE);
        this.rightClawServo.setDirection(Servo.Direction.FORWARD);
    }

    // Methods
    public void cycleSetPositionInState(ArmState state, int increment) {
        if (isInManualState()) disableManualState();
        if (currentState != state) {
            currentState = state;
            currentPositionInState = 0;
        }
        currentPositionInState = Math.max(Math.min(currentPositionInState + increment, state.getPositions().length-1), 0);
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
        extensionMotor.setTargetPosition(Math.max(0, Calculations.scaleToEncoderArmExtension(scaled)));
    }

    public void applyWristPosition(double scaled) {
        // The angle for the wrist to point at, on a scale where 1 is up
        wristServo.setPosition(Calculations.scaleToEncoderArmWrist(scaled));
    }

    public void alignWristToBackdrop() {
        alignWristToAngle(5.0 / 6.0);
    }

    public void alignWristToAngle(double scaled) {
        wristServo.setPosition(Calculations.alignArmWristToAngle(scaled, Calculations.encoderToScaleArmRotation(rotationMotor.getCurrentPosition())));
    }

    public void setRotationMotorPower(double power) {
        rotationMotor.setPower(power);
    }

    public void setExtensionMotorPower(double power) {
        extensionMotor.set(power);
    }

    public void setExtensionServoPower(double power) {
        extensionServo.setPower(power);
    }

    public void openLeftClaw() {
        leftClawServo.setPosition(Constants.LEFT_CLAW_OPEN);
    }

    public void openRightClaw() {
        rightClawServo.setPosition(Constants.RIGHT_CLAW_OPEN);
    }

    public void openPartlyLeftClaw() {
        leftClawServo.setPosition(Constants.LEFT_CLAW_PARTLY);
    }

    public void openPartlyRightClaw() {
        rightClawServo.setPosition(Constants.RIGHT_CLAW_PARTLY);
    }

    public void closeLeftClaw() {
        leftClawServo.setPosition(Constants.LEFT_CLAW_CLOSED);
    }

    public void closeRightClaw() {
        rightClawServo.setPosition(Constants.RIGHT_CLAW_CLOSED);
    }

    public void toggleLeftClaw() {
        /* Servos don't actually tell their position to the Control Hub. getPosition() is always
         equivalent to the last value of setPosition(), so this is an acceptable way to tell if the
         claw is open or closed. This wouldn't work with real encoders since no motor is 100% accurate. */
        if (leftClawServo.getPosition() == Constants.LEFT_CLAW_CLOSED) {
            openLeftClaw();
        } else {
            closeLeftClaw();
        }
    }

    public void toggleRightClaw() {
        if (rightClawServo.getPosition() == Constants.RIGHT_CLAW_CLOSED) {
            openRightClaw();
        } else {
            closeRightClaw();
        }
    }

    // Open both claws if both claws are already closed, otherwise close both.
    public void toggleBothClaws() {
        if (leftClawServo.getPosition() == Constants.LEFT_CLAW_CLOSED && rightClawServo.getPosition() == Constants.RIGHT_CLAW_CLOSED) {
            openLeftClaw();
            openRightClaw();
        } else {
            closeLeftClaw();
            closeRightClaw();
        }
    }

    public void enableManualState() {
        currentState = ArmState.MANUAL;
        currentPositionInState = 0;

        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setPower(0);

        extensionMotor.set(0);
        extensionMotor.setRunMode(Motor.RunMode.RawPower);
    }

    public void disableManualState() {
        currentState = ArmState.UNKNOWN;
        currentPositionInState = 0;

        rotationMotor.setPower(Constants.ARM_ROTATION_POWER);
        rotationMotor.setTargetPosition(0);
        rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extensionMotor.setTargetPosition(extensionMotor.getCurrentPosition());
        extensionMotor.setRunMode(Motor.RunMode.PositionControl);
        extensionMotor.set(Constants.ARM_EXTENSION_POWER);
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
        extensionMotor.resetEncoder();
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

    public Servo getLeftClawServo() {
        return leftClawServo;
    }

    public Servo getRightClawServo() {
        return rightClawServo;
    }
}
