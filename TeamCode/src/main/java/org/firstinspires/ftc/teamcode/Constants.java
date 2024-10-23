package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

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
    public static final String  NAME_DRIVE_FL = "drive_front_left";
    public static final String  NAME_DRIVE_FR = "drive_front_right";
    public static final String  NAME_DRIVE_BL = "drive_back_left";
    public static final String  NAME_DRIVE_BR = "drive_back_right";

    public static final String  NAME_INTAKE = "intake_servo";

    public static final String  NAME_ARM_ROTATE = "arm_rotation_motor";
    public static final String  NAME_ARM_EXTEND_M = "arm_extension_motor";
    public static final String  NAME_ARM_WRIST = "arm_wrist_servo";

    public static final String  NAME_IMU = "imu";

    // Component configuration
    public static final double    INTAKE_POWER               = 1.0;  // The power (speed) at which the intake moves while intaking on a scale of 0 to 1
    public static final double    OUTTAKE_POWER              = 1.0;  // The power (speed) at which the intake moves while outtaking on a scale of 0 to 1

    public static final double    WRIST_POSITION_UP          = 0.62; // The servo position for the wrist when the arm is down the wrist is pointing straight up

    public static final int       EXTENSION_TICKS_1_INCH     = 32;   // The number of encoder ticks for the arm extension to extend 1 inch

    public static final int       ROTATION_TICKS_180_DEGREES     = (2450 - 900) * 2; // The number of encoder ticks for the arm rotation to travel 180 degrees
    public static final int       ROTATION_TICKS_NORTH           = 2450;     // The number of arm rotation encoder ticks at the north (straight up) position

    public static final double    DRIVE_TICKS_REVOLUTION     = 537.6;  // The number of encoder ticks in one revolution of a drive motor
    public static final double    WHEEL_CIRCUMFERENCE_INCHES = (96 / 25.4) * Math.PI; // The approximate circumference of a drive motor's wheel
                                                                              // Wheels are 96mm in diameter, this must be converted to inches

    // TeleOp configuration
    public static final double    DRIVE_POWER_MULTIPLIER      = 1.0; // The multiplier scale on the robot's drivetrain power
    public static final double    DRIVE_POWER_MULTIPLIER_MED  = 0.6; // The multiplier scale on the robot's drivetrain power (when going medium speed)
    public static final double    DRIVE_POWER_MULTIPLIER_SLOW = 0.4; // The multiplier scale on the robot's drivetrain power (when going slow)
    public static final double    ARM_ROTATION_POWER          = 0.6; // The amount of power that the arm rotates with
    public static final double    ARM_EXTENSION_POWER         = 0.4; // The amount of power that the arm extends/retracts with

}
