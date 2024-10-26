package org.firstinspires.ftc.teamcode.helper;

/*
 * This is a class which represents a position for the arm assembly to go to.
 * It records target positions for the three parts of the arm. Positions are recorded as numbers on
 * a scale, not directly as encoder positions.
 * Arm positions also record what state the intake is in (intaking, outtaking, stopped).
 *
 * When a position is applied, it is automatically converted from the scale to the equivalent
 * encoder position. This way, positions won't have to be redone whenever a change is made that
 * affects encoder readings; as long as the constants class is configured correctly, the same value
 * on the scale will always correspond to the same physical position on the robot.
 */
public class ArmPosition {

    // Private instance variables
    private final double rotationAngle; // The angle for the rotation to point at, on a scale of 0 (straight down) to 1 (straight up)
    private final double extensionPosition;
    private final double raisePosition;
    private final double wristAngle; // The angle for the wrist to point at, on a scale where 1 is up
    private final IntakeState intakeState; // What the intake should do at this position (intake, outtake, stop)

    // Constructor method
    public ArmPosition(double rotationAngle, double extensionPosition, double raisePosition, double wristAngle) {
        this(rotationAngle, extensionPosition, raisePosition, wristAngle, IntakeState.NONE);
    }

    public ArmPosition(double rotationAngle, double extensionPosition, double raisePosition, double wristAngle, IntakeState intakeState) {
        this.rotationAngle = rotationAngle;
        this.extensionPosition = extensionPosition;
        this.raisePosition = raisePosition;
        this.wristAngle = wristAngle;
        this.intakeState = intakeState;
    }

    // Getter methods
    public double getRotationAngle() {
        return rotationAngle;
    }

    public double getExtensionPosition() {
        return extensionPosition;
    }

    public double getRaisePosition() {
        return raisePosition;
    }

    public double getWristAngle() {
        return wristAngle;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }
}
