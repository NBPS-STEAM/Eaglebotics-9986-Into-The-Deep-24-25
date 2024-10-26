package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.helper.QuadMotorValues;

/*
 * This is a class that contains methods to perform specific calculations.
 * Keeping the calculations separate from the rest of the code keeps things from getting too messy,
 * and allows complex calculations to be reused easily.
 *
 * This class contains static methods. Static methods are a part of the class, not objects made from it.
 * Static methods are called directly from the class's name.
 */
public class Calculations {

    // Mecanum drive calculations
    public static QuadMotorValues<Double> mecanumDriveRobotCentric(double axial, double lateral, double yaw) {
        // Combine the direction requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level.

        //lateral *= 1.1; // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);

        double frontLeftPower = (axial + lateral + yaw) / denominator;
        double frontRightPower = (axial - lateral - yaw) / denominator;
        double backLeftPower = (axial - lateral + yaw) / denominator;
        double backRightPower = (axial + lateral - yaw) / denominator;

        // Send calculated power to wheels
        return new QuadMotorValues<>(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    public static QuadMotorValues<Double> mecanumDriveFieldCentric(double axial, double lateral, double yaw, double heading) {
        // Rotate the movement direction counter to the robot's rotation
        double rotLateral = lateral * Math.cos(-heading) - axial * Math.sin(-heading);
        double rotAxial = lateral * Math.sin(-heading) + axial * Math.cos(-heading);

        //rotLateral *= 1.1;  // Counteract imperfect strafing

        // Combine the direction requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level.

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotAxial) + Math.abs(rotLateral) + Math.abs(yaw), 1);

        double frontLeftPower = (rotAxial + rotLateral + yaw) / denominator;
        double frontRightPower = (rotAxial - rotLateral - yaw) / denominator;
        double backLeftPower = (rotAxial - rotLateral + yaw) / denominator;
        double backRightPower = (rotAxial + rotLateral - yaw) / denominator;

        // Send calculated power to wheels
        return new QuadMotorValues<>(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    // Position scaling calculations
    // The distance for a drive motor to spin to, on a scale of inches
    public static int inchesToEncoderDrive(double inches) {
        return (int) (inches * (Constants.DRIVE_TICKS_REVOLUTION / Constants.WHEEL_CIRCUMFERENCE_INCHES));
    }
    public static double encoderToInchesDrive(int encoder) {
        return (encoder * Constants.WHEEL_CIRCUMFERENCE_INCHES) / Constants.DRIVE_TICKS_REVOLUTION;
    }

    // The angle for the rotation to point at, on a scale of 0 (straight down) to 1 (straight up)
    // Remember that the rotation motor's zero position will likely be above 0 on this scale
    public static int scaleToEncoderArmRotation(double scale) {
        return (int) (scale * Constants.ROTATION_TICKS_180_DEGREES - (Constants.ROTATION_TICKS_180_DEGREES - Constants.ROTATION_TICKS_NORTH));
    }
    public static double encoderToScaleArmRotation(int encoder) {
        return (double) (encoder + (Constants.ROTATION_TICKS_180_DEGREES - Constants.ROTATION_TICKS_NORTH)) / Constants.ROTATION_TICKS_180_DEGREES;
    }

    // The position for the extension to travel to, on a scale of inches
    public static int scaleToEncoderArmExtension(double scale) {
        return (int) (scale * Constants.EXTENSION_TICKS_1_INCH);
    }
    public static int scaleToEncoderArmRaise(double scale) {
        return (int) (scale * Constants.RAISE_TICKS_1_INCH);
    }
    public static double encoderToScaleArmExtension(int encoder) {
        return (double) encoder / Constants.EXTENSION_TICKS_1_INCH;
    }

    public static double encoderToScaleArmRaise(int encoder) {
        return (double) encoder / Constants.RAISE_TICKS_1_INCH;
    }

    // The angle for the wrist to point at, on a scale where 1 is up
    public static double scaleToEncoderArmWrist(double scale) {
        return scale - (1 - Constants.WRIST_POSITION_UP);
    }
    public static double encoderToScaleArmWrist(double encoder) {
        return encoder + (1 - Constants.WRIST_POSITION_UP);
    }
}
