package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This is a class that contains numbers to be used elsewhere in the code.
 * Keeping the constant numbers separate from the rest of the code makes it easier to configure
 * properties of the robot and keeps things from getting too messy.
 *
 * This class contains static variables. Static variables are a part of the class, not objects made from it.
 * Static variables are used directly from the class's name.
 *
 * This class contains final variables. Final variables cannot be changed by = after being created.
 * If a final variable is a primitive type (such as int or boolean), it cannot be changed at all after being created.
 * Final variables are useful for numbers that need to be used in multiple places but shouldn't change.
 */
@Config
public class Constants {

    // Hardware names
    public static final String  NAME_DRIVE_FL = "frontLeftMotor";
    public static final String  NAME_DRIVE_FR = "frontRightMotor";
    public static final String  NAME_DRIVE_BL = "backLeftMotor";
    public static final String  NAME_DRIVE_BR = "backRightMotor";

    public static final String  NAME_INTAKE = "intakeServo";

    public static final String  NAME_ARM_ROTATE = "rotationMotor";
    public static final String  NAME_ARM_EXTEND = "extensionMotor";
    public static final String  NAME_ARM_RAISE = "raiseMotor";
    public static final String  NAME_ARM_WRIST = "wristServo";
    public static final String  NAME_ARM_COLOR_RANGE = "colorRangeSensor";

    public static final String  NAME_IMU = "imu";

    // Hardware Directions
    public static final DcMotor.Direction DIRECTION_DRIVE_FL = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction DIRECTION_DRIVE_FR = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction DIRECTION_DRIVE_BL = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction DIRECTION_DRIVE_BR = DcMotor.Direction.REVERSE;

    public static final Servo.Direction   DIRECTION_INTAKE   = Servo.Direction.FORWARD;

    public static final DcMotor.Direction DIRECTION_ARM_ROTATE = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction DIRECTION_ARM_EXTEND = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction DIRECTION_ARM_RAISE  = DcMotor.Direction.REVERSE;
    public static final Servo.Direction   DIRECTION_ARM_WRIST  = Servo.Direction.FORWARD;

    public static final RevHubOrientationOnRobot.LogoFacingDirection
            IMU_HUB_LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
    public static final RevHubOrientationOnRobot.UsbFacingDirection
            IMU_HUB_USB_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    // Component configuration
    public static final double    WRIST_POSITION_UP          = 0.234;  // The servo position of the wrist when it's pointing straight forward

    public static final int       EXTENSION_TICKS_1_INCH     = 32;   // (PLACEHOLDER) The number of encoder ticks for the arm extension to extend 1 inch
    public static final int       RAISE_TICKS_1_INCH         = 32;   // (PLACEHOLDER) The number of encoder ticks for the arm raise to extend 1 inch

    public static final int       ROTATION_TICKS_180_DEGREES = (1650 - 650) * 2; // The number of encoder ticks for the arm rotation to travel 180 degrees
    public static final int       ROTATION_TICKS_NORTH       = 1650;     // The number of arm rotation encoder ticks at the north (straight up) position

    public static final double    DRIVE_TICKS_REVOLUTION     = 537.6;  // The number of encoder ticks in one revolution of a drive motor
    public static final double    WHEEL_CIRCUMFERENCE_INCHES = (96 / 25.4) * Math.PI; // The approximate circumference of a drive motor's wheel
                                                                              // Wheels are 96mm in diameter, this must be converted to inches

    // TeleOp configuration
    public static final double    DRIVE_POWER_MULTIPLIER      = 1.0; // The multiplier scale on the robot's drivetrain power
    public static final double    DRIVE_POWER_MULTIPLIER_MED  = 0.4; // The multiplier scale on the robot's drivetrain power (when going medium speed)
    public static final double    DRIVE_POWER_MULTIPLIER_SLOW = 0.25; // The multiplier scale on the robot's drivetrain power (when going slow)

    public static final double    ARM_ROTATION_POWER          = 0.6; // The amount of power that the arm rotates with
    public static final double    ARM_EXTENSION_POWER         = 0.6; // The amount of power that the arm extends/retracts with
    public static final double    ARM_RAISE_POWER             = 0.6; // The amount of power that the arm rises/lowers with

    public static final double    ARM_ROTATION_POWER_MANUAL   = 0.5; // The amount of power that the arm rotates with (MANUAL CONTROL)
    public static final double    ARM_EXTENSION_POWER_MANUAL  = 0.5; // The amount of power that the arm extends/retracts with (MANUAL CONTROL)
    public static final double    ARM_RAISE_POWER_MANUAL      = 0.5; // The amount of power that the arm rises/lowers with (MANUAL CONTROL)

    public static final double    INTAKE_PRIME_POSITION       = 0.29; // The position of the intake when primed to intake on a scale of 0 to 1
    public static final double    INTAKE_POSITION             = 1.0; // The position of the intake when intaking (closing) on a scale of 0 to 1
    public static final double    OUTTAKE_POSITION            = 0.29; // The position of the intake when outtaking (opening) on a scale of 0 to 1

    public static final double    INTAKE_SAMPLE_THRESHOLD     = 30.0; // (IN MILLIMETERS) If the range sensor in the intake measures less than this when looking for a sample, then it has it.
    public static final double    INTAKE_SPECIMEN_THRESHOLD   = 20.0; // (IN MILLIMETERS) If the range sensor in the intake measures less than this when looking for a specimen, then it has it.

    public static final double    STICK_COMMAND_THRESHOLD     = 0.8;  // If a controller joystick moves father than this, it can trigger a command.


    // Testing TeleOp configuration
    public static final int       TESTING_SAVE_STATE_SLOTS    = 2;    // The number of slots that a device can store its state to in testing mode
    public static final double    TESTING_MODE_CHANGE_THRESHOLD = 0.2;// The absolute distance that a joystick must move for move/offset mode to switch in testing mode

    public static final double    TESTING_DCMOTOR_OFFSET_RATE = 5.0;  // The rate at which a DC motor's position will be changed when offsetting in testing mode
    public static final double    TESTING_SERVO_OFFSET_RATE   = 0.02; // The rate at which a DC motor's position will be changed when offsetting in testing mode

}
