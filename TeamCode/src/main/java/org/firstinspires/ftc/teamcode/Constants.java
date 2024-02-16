package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.helper.SidePosition;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

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

    public static final String  NAME_CLAW_L = "left_claw_servo";
    public static final String  NAME_CLAW_R = "right_claw_servo";

    public static final String  NAME_ARM_ROTATE = "arm_rotation_motor";
    public static final String  NAME_ARM_EXTEND_M = "arm_extension_motor";
    public static final String  NAME_ARM_EXTEND_S = "arm_extension_servo";
    public static final String  NAME_ARM_WRIST = "arm_wrist_servo";

    public static final String  NAME_IMU = "imu";

    // Control configuration
    public static final double    ARM_CONTROL_STICK_THRESHOLD = 0.8; // How far the stick controlling the arm must be from its center for the arm to move

    // Component configuration
    public static final double    LEFT_CLAW_CLOSED           = 0.77; // The position of the left claw when it's fully closed
    public static final double    LEFT_CLAW_PARTLY           = 0.59; // The position of the left claw when it's partly open
    public static final double    LEFT_CLAW_OPEN             = 0.30; // The position of the left claw when it's fully open
    public static final double    RIGHT_CLAW_CLOSED          = 0.77; // The position of the right claw when it's fully closed
    public static final double    RIGHT_CLAW_PARTLY          = 0.59; // The position of the right claw when it's partly open
    public static final double    RIGHT_CLAW_OPEN            = 0.30; // The position of the right claw when it's fully open

    public static final double    WRIST_POSITION_UP          = 0.62; // The servo position for the wrist when the arm is down the wrist is pointing straight up

    public static final int       EXTENSION_TICKS_1_INCH     = 32;   // The number of encoder ticks for the arm extension to extend 1 inch

    public static final int       ROTATION_TICKS_180_DEGREES     = (2450 - 900) * 2; // The number of encoder ticks for the arm rotation to travel 180 degrees
    public static final int       ROTATION_TICKS_NORTH           = 2450;     // The number of arm rotation encoder ticks at the north (straight up) position
    public static final double    ROTATION_POSITION_MAX          = 0.95;     // The safe maximum number of arm rotation encoder ticks for the arm to swing between
    public static final int       ROTATION_POSITION_MAX_OVERRIDE = 4600;     // The absolute maximum number of encoder ticks for the arm rotation to swing between

    public static final double    DRIVE_TICKS_REVOLUTION     = 537.6;  // The number of encoder ticks in one revolution of a drive motor
    public static final double    WHEEL_CIRCUMFERENCE_INCHES = (96 / 25.4) * Math.PI; // The approximate circumference of a drive motor's wheel
                                                                              // Wheels are 96mm in diameter, this must be converted to inches

    // TeleOp configuration
    public static final double    DRIVE_POWER_MULTIPLIER      = 1.0; // The multiplier scale on the robot's drivetrain power
    public static final double    DRIVE_POWER_MULTIPLIER_MED  = 0.6; // The multiplier scale on the robot's drivetrain power (when going medium speed)
    public static final double    DRIVE_POWER_MULTIPLIER_SLOW = 0.4; // The multiplier scale on the robot's drivetrain power (when going slow)
    public static final double    ARM_ROTATION_POWER          = 0.6; // The amount of power that the arm rotates with
    public static final double    ARM_EXTENSION_POWER         = 0.4; // The amount of power that the arm extends/retracts with
    public static final double    MANUAL_ROTATION_COARSE_POWER  = 0.8; // The amount of power that the arm rotates with when controlled manually (coarse adjustment)
    public static final double    MANUAL_ROTATION_FINE_POWER    = 0.2; // The amount of power that the arm rotates with when controlled manually (fine adjustment)
    public static final double    MANUAL_EXTENSION_COARSE_POWER = 0.8; // The amount of power that the arm extends with when controlled manually (coarse adjustment)
    public static final double    MANUAL_EXTENSION_FINE_POWER   = 0.2; // The amount of power that the arm extends with when controlled manually (fine adjustment)

    // Autonomous configuration
    public static final SidePosition FALLBACK_PROP_POSITION = SidePosition.CENTER; // The prop position to fall back to if one cannot be found

    // Autonomous vision configuration
    public static final int   VISION_CAMERA_WIDTH  = 640;
    public static final int   VISION_CAMERA_HEIGHT = 480;
    public static       Point VISION_REGION1_CENTER_ANCHOR_POINT = new Point(460,305); // The center of the area of the right prop in the image at the start of auto
    public static       Point VISION_REGION2_CENTER_ANCHOR_POINT = new Point(150,290); // The center of the area of the middle prop in the image at the start of auto
    public static       int   VISION_SUBREGION_WIDTH    = 20;  // The width of the area to look for a prop in the vision image
    public static       int   VISION_SUBREGION_HEIGHT   = 30;  // The height of the area to look for a prop in the vision image
    public static       int   VISION_REGION1_SUBREGION_COUNT     = 3;   // The number of subregions to look for the right prop
    public static       int   VISION_REGION2_SUBREGION_COUNT     = 5;   // The number of subregions to look for the middle prop
    public static       int   VISION_SUBREGION_DISTANCE = 15;  // The distance between each subregion
    public static       int   VISION_CHROMA_THRESHOLD   = 155; // The minimum chroma value for one of two chroma values to be likely to have a prop

    // Autonomous vision display configuration
    public static final Scalar    BLUE  = new Scalar(0, 0, 255); // The color of boxes drawn blue on the vision display
    public static final Scalar    GREEN = new Scalar(0, 255, 0); // The color of boxes drawn green on the vision display
    public static final int       VISION_BOX_THICKNESS = 4;      // The thickness of boxes drawn on the vision display

}
