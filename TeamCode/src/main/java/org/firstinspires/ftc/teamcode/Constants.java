package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.helper.Geo;

import java.util.HashMap;
import java.util.Map;

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

    public static final String[] NAMES_INTAKE = new String[]{"intakeServo1", "intakeServo2"};

    public static final String  NAME_ARM_ROTATE = "rotationMotor";
    public static final String  NAME_ARM_EXTEND = "extensionMotor";
    public static final String  NAME_ARM_RAISE = "raiseMotor";
    public static final String  NAME_ARM_WRIST = "wristServo";
    public static final String[] NAMES_ARM_COLOR_RANGE = new String[]{"colorRangeSensor1"};
    public static final String  NAME_ARM_RETRACT = "retractMotor";

    public static final String  NAME_IMU = "imu1";

    public static final String  NAME_CAMERA = "webcam1";

    // Hardware Directions
    public static final DcMotor.Direction DIRECTION_DRIVE_FL = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction DIRECTION_DRIVE_FR = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction DIRECTION_DRIVE_BL = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction DIRECTION_DRIVE_BR = DcMotor.Direction.FORWARD;

    public static final CRServo.Direction[] DIRECTIONS_INTAKE  = new CRServo.Direction[]{CRServo.Direction.FORWARD, CRServo.Direction.REVERSE};

    public static final DcMotor.Direction DIRECTION_ARM_ROTATE = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction DIRECTION_ARM_EXTEND = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction DIRECTION_ARM_RAISE  = DcMotor.Direction.REVERSE;
    public static final Servo.Direction   DIRECTION_ARM_WRIST  = Servo.Direction.FORWARD;
    public static final DcMotor.Direction DIRECTION_ARM_RETRACT  = DcMotor.Direction.REVERSE;
    public static final DcMotor.ZeroPowerBehavior ZEROPOWER_ARM_RETRACT  = DcMotor.ZeroPowerBehavior.FLOAT;

    public static final RevHubOrientationOnRobot.LogoFacingDirection
            IMU_HUB_LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static final RevHubOrientationOnRobot.UsbFacingDirection
            IMU_HUB_USB_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    // Component configuration
    public static final double    WRIST_POSITION_UP          = 0.334;  // The servo position of the wrist when it's pointing straight forward

    public static final int       EXTENSION_TICKS_1_INCH     = 32;   // (PLACEHOLDER) The number of encoder ticks for the arm extension to extend 1 inch
    public static final int       RAISE_TICKS_1_INCH         = 32;   // (PLACEHOLDER) The number of encoder ticks for the arm raise to extend 1 inch

    public static final int       ROTATION_TICKS_180_DEGREES = (1650 - 650) * 2; // The number of encoder ticks for the arm rotation to travel 180 degrees
    public static final int       ROTATION_TICKS_NORTH       = 1650;     // The number of arm rotation encoder ticks at the north (straight up) position

    public static       boolean   SENSOR_CR_LED_ENABLED       = true; // Whether the LED is enabled on ColorRangeSensors (this might not do anything?)
    public static       float     SENSOR_CR_GAIN              = 11.0f; // The gain on ColorRangeSensors

    public static final double    DRIVE_TICKS_REVOLUTION     = 537.6;  // The number of encoder ticks in one revolution of a drive motor
    public static final double    WHEEL_CIRCUMFERENCE_INCHES = (96 / 25.4) * Math.PI; // The approximate circumference of a drive motor's wheel
                                                                              // Wheels are 96mm in diameter, this must be converted to inches

    // TeleOp configuration
    public static final double    DRIVE_POWER_MULTIPLIER      = 1.0; // The multiplier scale on the robot's drivetrain power
    public static final double    DRIVE_POWER_MULTIPLIER_MED  = 0.4; // The multiplier scale on the robot's drivetrain power (when going medium speed)
    public static final double    DRIVE_POWER_MULTIPLIER_SLOW = 0.25; // The multiplier scale on the robot's drivetrain power (when going slow)
    public static       double    DRIVE_TOUCHPAD_STRAFE_SENSITIVITY = 5.0; // The sensitivity of touchpad driving in strafing
    public static       double    DRIVE_TOUCHPAD_TURN_SENSITIVITY   = 5.0; // The sensitivity of touchpad driving in turning

    public static final double    ARM_ROTATION_POWER          = 0.5; // The amount of power that the arm rotates with in teleop
    public static final double    ARM_EXTENSION_POWER         = 0.8; // The amount of power that the arm extends/retracts with in teleop
    public static final double    ARM_RAISE_POWER             = 1.0; // The amount of power that the arm rises with
    public static final double    ARM_RETRACT_POWER           = 1.0; // The amount of power that the arm lowers with (only used for hang)

    public static final double    ARM_ROTATION_POWER_MANUAL   = 0.8; // (NOT USED IN LATEST) The amount of power that the arm rotates with (MANUAL CONTROL)
    public static final double    ARM_EXTENSION_POWER_MANUAL  = 0.5; // (NOT USED IN LATEST) The amount of power that the arm extends/retracts with (MANUAL CONTROL)
    public static final double    ARM_RAISE_POWER_MANUAL      = 0.5; // (NOT USED IN LATEST) The amount of power that the arm rises/lowers with (MANUAL CONTROL)
    public static final double    ARM_WRIST_RATE_MANUAL       = 0.03; // The rate that the wrist turns at while manually offsetting it (MANUAL CONTROL)

    public static final double    INTAKE_POWER                = 1.0; // The power of the intake when intaking on a scale of -1 to 1
    public static final double    OUTTAKE_POWER               = -0.3; // The power of the intake when outtaking on a scale of -1 to 1

    public static final int       ROTATION_TARGET_THRESHOLD   = 10;  // The maximum distance of the rotation from its target to consider it on-target
    public static final int       EXTENSION_TARGET_THRESHOLD  = 10;  // The maximum distance of the extension from its target to consider it on-target
    public static final int       RAISE_TARGET_THRESHOLD      = 10;  // The maximum distance of the raise from its target to consider it on-target

    public static final double    INTAKE_SAMPLE_THRESHOLD     = 17.0; // (IN MILLIMETERS) If the range sensor in the intake measures less than this, then it has a sample.

    public static final double    ANALOG_COMMAND_THRESHOLD     = 0.25;  // If an analog controller input moves farther than this, it may trigger a command.
    public static final double    STICK_INTERRUPT_DEADZONE_SQR = 0.15*0.15; // If either of the base driver's joysticks move more than this, any running autonomous driving command will be interrupted (squared)


    // Auto configuration
    public static final double    ARM_ROTATION_POWER_AUTO     = 0.4; // The amount of power that the arm rotates with in the autonomous routine (some routines override this)
    public static final double    ARM_EXTENSION_POWER_AUTO    = 0.4; // The amount of power that the arm extends/retracts with in the autonomous routine (some routines override this)
    public static final double    ARM_RAISE_POWER_AUTO        = 1.0; // The amount of power that the arm rises/lowers with in the autonomous routine (some routines override this)

    public static       double    AUTO_DRIVE_VEL_MAX          = 40.0; // The default maximum velocity of autonomous driving using DriveSubsystemRRVision
    public static       double    AUTO_DRIVE_ACCEL_MAX        = 20.0; // The default maximum acceleration of autonomous driving using DriveSubsystemRRVision
    public static       double    AUTO_DRIVE_DECEL_MAX        = -20.0; // The default maximum deceleration of autonomous driving using DriveSubsystemRRVision
    public static       double    AUTO_DRIVE_ANG_VEL_MAX      = Math.PI; // The default maximum angular velocity of autonomous driving using DriveSubsystemRRVision
    public static       double    AUTO_DRIVE_ANG_ACCEL_MAX    = Math.PI; // The default maximum angular acceleration of autonomous driving using DriveSubsystemRRVision

    public static       double    AUTO_DRIVE_POS_TOLERANCE    = 1.5; // The positional tolerance of autonomous driving using DriveSubsystemRRVision
    public static       double    AUTO_DRIVE_VEL_TOLERANCE    = 2; // The velocity tolerance of autonomous driving using DriveSubsystemRRVision
    public static       double    AUTO_DRIVE_ANG_TOLERANCE    = Math.toRadians(4); // The angular tolerance of autonomous driving using DriveSubsystemRRVision
    public static       double    AUTO_DRIVE_ANGVEL_TOLERANCE = Math.toRadians(4); // The angular velocity tolerance of autonomous driving using DriveSubsystemRRVision
    public static       double    AUTO_DRIVE_TOLERANCE_TIMEOUT = 2.0; // The maximum time that can be spent correcting the end of a path in autonomous driving using DriveSubsystemRRVision

    public static final double    LOCALIZATION_VISION_RANGE         = 60.0; // (inches) AprilTags beyond this distance will be ignored in vision localization.
    //public static       double    LOCALIZATION_LINEAR_THRESHOLD     = 50.0; // The maximum speed at which the camera can be used to localize
    //public static       double    LOCALIZATION_ANGULAR_THRESHOLD    = Math.PI * 0.6; // The maximum angular speed at which the camera can be used to localize

    // Filter constants for de-noising the absolute localizer in DriveSubsystemRRVision
    // See the "Least Squares + Kalman Filter" section for more info: https://www.ctrlaltftc.com/homeostasis-by-thermal-equilibrium/state-estimation-and-filters
    public static       boolean ABS_LOCALIZER_DENOISE   = false; // Whether to use a Kalman filter to de-noise the results from absolute position localizer results (i.e. vision)
    public static       double  ABS_LOCALIZER_DENOISE_Q = 0.3; // High values put more emphasis on the sensor.
    public static       double  ABS_LOCALIZER_DENOISE_R = 3.0; // High Values put more emphasis on regression.
    public static       int     ABS_LOCALIZER_DENOISE_N = 5;   // The number of estimates in the past we perform regression on.


    // Testing TeleOp configuration
    public static final int       TESTING_SAVE_STATE_SLOTS    = 2;    // The number of slots that a device can store its state to in testing mode
    public static final double    TESTING_MODE_CHANGE_THRESHOLD = 0.2;// The absolute distance that a joystick must move for move/offset mode to switch in testing mode

    public static final double    TESTING_DCMOTOR_OFFSET_RATE = 5.0;  // The rate at which a DC motor's position will be changed when offsetting in testing mode
    public static final double    TESTING_SERVO_OFFSET_RATE   = 0.02; // The rate at which a DC motor's position will be changed when offsetting in testing mode


    // Camera configuration
    /**
     * Variables to store the position and orientation of the camera on the robot. Setting these
     * values requires a definition of the axes of the camera and robot:
     *
     * Camera axes:
     * Origin location: Center of the lens
     * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
     *
     * Robot axes (this is typical, but you can define this however you want):
     * Origin location: Center of the robot at field height
     * Axes orientation: +x right, +y forward, +z upward
     *
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     *
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */
    // The following pose values are used for the VisionPortal's built-in robot localizer:
    public static final Position           CAM_POSITION    = new Position(DistanceUnit.INCH, 7.0, 2.125, 4.25, 0);
    public static final YawPitchRollAngles CAM_ORIENTATION = new YawPitchRollAngles(AngleUnit.DEGREES, -90, -90, 0, 0);
    // The following pose values are used for the custom 2D gyro robot localizer:
    public static final Geo.Vector2D       CAM_POSITION_2D     = new Geo.Vector2D(7.0, 2.125); // Camera position from center of robot relative to robot orientation
    public static final double             CAM_ORIENTATION_2D  = -Math.PI / 2; // The camera's yaw relative to robot orientation. Measured in radians, use Math.toRadians(degrees) if you prefer
    // End of pose config
    public static       boolean            CAM_DO_STREAM       = true; // Whether to stream the camera to the Driver Hub in non-testing vision opmodes
    public static       boolean            USE_GYRO_ESTIMATOR  = true; // Whether to use the custom 2D gyro-vision robot localizer in DriveSubsystemRRVision
    public static       int                CAM_SIZE_X      = 1280; // Width (pixels) of the camera's resolution (must be a supported resolution)
    public static       int                CAM_SIZE_Y      = 720;  // Height (pixels) of the camera's resolution (must be a supported resolution)
    public static       float              CAM_DECIMATION  = 3; // To be honest, I'm not sure what this does. More info in VisionPortalSubsystem.initAprilTag()
    public static       long               CAM_EXPOSURE_MS = 600; // (MICROSECONDS) The preferred exposure time of the camera

    // Camera tuning
    // For new global-shutter camera with black 3D-printed case:
    // TODO: label the model of camera on the case
    public static       double             CAM_FX          = 906.797;
    public static       double             CAM_FY          = 906.797;
    public static       double             CAM_CX          = 613.192;
    public static       double             CAM_CY          = 341.331;
    // For old Logitech webcam with white 3D-printed case:
    //public static       double             CAM_FX          = 956.17;
    //public static       double             CAM_FY          = 956.17;
    //public static       double             CAM_CX          = 438.219;
    //public static       double             CAM_CY          = 244.697;


    // Field-relative positions (based on AprilTag localization)

    // Only used for custom 2D gyro localizer:
    public static       double FIELD_HALFSIZE_X = 71;
    public static       double FIELD_HALFSIZE_Y = 71;
    public static final Map<Integer, Geo.Vector2D> POS_APRIL_TAGS = new HashMap<Integer, Geo.Vector2D>() {{
        put(11, new Geo.Vector2D(-FIELD_HALFSIZE_X, FIELD_HALFSIZE_Y * 2.0 / 3.0));
        put(12, new Geo.Vector2D(0, FIELD_HALFSIZE_Y));
        put(13, new Geo.Vector2D(FIELD_HALFSIZE_X, FIELD_HALFSIZE_Y * 2.0 / 3.0));
        put(14, new Geo.Vector2D(FIELD_HALFSIZE_X, -FIELD_HALFSIZE_Y * 2.0 / 3.0));
        put(15, new Geo.Vector2D(0, -FIELD_HALFSIZE_Y));
        put(16, new Geo.Vector2D(-FIELD_HALFSIZE_X, -FIELD_HALFSIZE_Y * 2.0 / 3.0));
    }};

    // These are the positions for the RED alliance and will be automatically mirrored for the blue alliance.
    //public static final Pose2d POS_BASKETS_APPROACH     = new Pose2d(-47.68, -40, Math.toRadians(-90));
    public static final Pose2d POS_BASKETS_SCORE        = new Pose2d(-55.75, -53.25, Math.toRadians(-140));
    public static final Pose2d POS_INTAKE_APPROACH      = new Pose2d(-40, 0, Math.toRadians(-90));

    public static final Pose2d POS_SPECIMEN_APPROACH    = new Pose2d(35, -42.8, Math.toRadians(-43.5));
    public static final Pose2d POS_SPECIMEN_INTAKE      = new Pose2d(44.3+2, -52.62-2, Math.toRadians(-43.5));

}
