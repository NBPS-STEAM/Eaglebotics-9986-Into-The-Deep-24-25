package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.helper.ArmPosition;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.helper.SidePosition;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * Autonomous! Awesome!
 *
 * <p>This routine uses vision recognition to detect a prop, red or blue depending on team color,
 * then approaches it using either a linear path or a spline while raising the arm, then finally
 * drives 20 inches in each direction towards the center of the field from where it started.</p>
 *
 * <p>Recently, Road Runner was majorly rewritten into Road Runner 1.0.
 * This new version is not backwards-compatible at all.
 * If you look for advice online, advice from before mid-2024 is likely out of date.</p>
 *
 * See <a href="https://rr.brott.dev/docs/v1-0/tuning">the official Road Runner documentation.</a>
 */
public class AutonomousRoutine extends LinearOpMode {

    // Launcher

    // See SidePosition for info on enums.
    public enum Routine {
        LINEAR,
        SPLINE,
        TESTING,
    }

    // Protected variables are private variables that are accessible by subclasses and classes in the same "package".
    // Put simply, the package is the folder that the class is in. Classes are grouped by package.
    // These variables are used as settings for what the robot should do when the OpMode is run.
    // These variables can be changed by a launcher class to start the code with different settings.
    // See any of the classes in the launcher folder.
    protected boolean isBlueSide = false;
    protected boolean isLongDistance = false;
    protected Routine selectedRoutine = Routine.LINEAR;

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(false);

        // Initialize the robot through the auto routine
        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // Abort nicely if the code is to be stopped
        if (isStopRequested()) return;

        // Execute the selected auto routine
        switch (selectedRoutine) {
            case LINEAR:
                linearRoutine();
                break;

            case SPLINE:
                splineRoutine();
                break;

            case TESTING:
                testingRoutine();
                break;
        }

        // Wait until automatic termination after timer
        while (opModeIsActive()) {
            sleep(50);
        }
    }

    // End of launcher


    // Variables

    // Pre-programmed positions
    private final ArmPosition compactPosition = new ArmPosition(0.18, 0, 1);
    private final ArmPosition highPosition = new ArmPosition(0.5, 0, 0.3);

    // Working variables
    final ElapsedTime runtime = new ElapsedTime();
    private Boolean isCameraOpened = null;
    private SidePosition propPosition = SidePosition.UNDEFINED;

    // Hardware objects
    private OpenCvCamera frontCamera;
    private MecanumDrive mDrive;
    private ArmSubsystem armSubsystem;


    // Method to initialize the robot
    public void initialize() {
        // Initialize the motors/hardware. The names used here must correspond to the names set on the driver control station.
        WebcamName frontWebcam = hardwareMap.get(WebcamName.class, "front_webcam");
        int frontCameraViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Configure hardware

        // Camera
        frontCamera = OpenCvCameraFactory.getInstance().createWebcam(frontWebcam, frontCameraViewId);
        // The following will run asynchronously. Callback methods are provided to be run when the camera is, or fails to be, opened.
        frontCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            // This is run if/when the camera is opened successfully.
            @Override
            public void onOpened() {
                // Start camera frontCameraViewId
                frontCamera.startStreaming(Constants.VISION_CAMERA_WIDTH, Constants.VISION_CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                isCameraOpened = true;
            }

            // This is run if/when the camera fails to be opened.
            @Override
            public void onError(int errorCode) {
                // Show warning on the driver control station
                isCameraOpened = false;
                telemetry.addData("WARNING: FRONT CAMERA FAILED TO OPEN", "(code " + errorCode + ")");
                telemetry.update();
            }
        });

        // Drivetrain
        // This is Road Runner's special mecanum drivetrain class.
        mDrive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), 0.0));

        // Arm assembly
        armSubsystem = new ArmSubsystem(hardwareMap);
        armSubsystem.closeLeftClaw();
        armSubsystem.closeRightClaw();
        armSubsystem.applyPosition(compactPosition);
        armSubsystem.applyWristPosition(1);

        // Initialization complete
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /**
     * Method to run the routine for testing the autonomous vision.
     */
    public void testingRoutine() {
        // START
        telemetry.addData("Starting TESTING autonomous routine.", "");
        telemetry.addData("", "");
        telemetry.update();

        runtime.reset();

        // Wait until camera has opened
        while (isCameraOpened == null) sleep(50);

        // FIND PROP
        routineFindProp();
    }

    /**
     * Method to run the linear autonomous routine.
     * Reminder: angles in Road Runnner are measured in counter-clockwise radians.
     */
    public void linearRoutine() {
        // START
        telemetry.addData("Starting linear autonomous routine.", "");
        telemetry.addData("", "");
        telemetry.update();

        runtime.reset();

        // Wait until camera has opened
        while (isCameraOpened == null) sleep(50);

        // FIND PROP
        routineFindProp();

        // DRIVE TO PROP
        /*
         * At this point, the robot is still at the starting position.
         */
        routineLinearToProp();

        // DRIVE AWAY
        /*
         * At this point, the robot is beside the prop and is facing the prop.
         */
        routineLinearToAway();

        // COMPLETE
        telemetry.addData("Autonomous routine complete.", "( " + Math.ceil(runtime.seconds()) + " seconds)");
        telemetry.addData("Waiting for OpMode termination.", "");
        telemetry.update();
    }

    /**
     * Method to run the spline autonomous routine.
     * Reminder: angles in Road Runnner are measured in counter-clockwise radians.
     */
    public void splineRoutine() {
        // START
        telemetry.addData("Starting spline autonomous routine.", "");
        telemetry.update();

        runtime.reset();

        // Wait until camera has opened
        while (isCameraOpened == null) sleep(50);

        // FIND PROP
        routineFindProp();

        // DRIVE TO PROP
        /*
         * At this point, the robot is still at the starting position.
         */
        routineSplineToProp();

        // DRIVE AWAY
        /*
         * At this point, the robot is beside the prop and is facing the prop.
         */
        routineSplineToAway();

        // COMPLETE
        telemetry.addData("Autonomous routine complete.", "( " + Math.ceil(runtime.seconds()) + " seconds)");
        telemetry.addData("Waiting for OpMode termination.", "");
        telemetry.update();
    }


    private void routineFindProp() {
        routineFindProp(false);
    }

    private void routineFindProp(boolean isOutputtingChromaChannel) {
        // Find the prop
        if (isCameraOpened) {
            // If the camera opened...

            // Resume the front camera's viewport
            frontCamera.resumeViewport();

            // Start the prop vision pipeline
            telemetry.addData("Looking for prop...", "");
            telemetry.update();
            VisionPipelineDetectProp visionPipeline = new VisionPipelineDetectProp(isBlueSide, isOutputtingChromaChannel);
            frontCamera.setPipeline(visionPipeline);

            // Wait until the pipeline has returned a position
            double startTime = runtime.seconds();
            while (visionPipeline.getPropPosition() == SidePosition.UNDEFINED) {
                if (runtime.seconds() - startTime > 10) { // timeout after 10 seconds
                    telemetry.addData("Timeout on prop vision pipeline!", "(" + (runtime.seconds() - startTime) + " seconds)");
                    break;
                }
                sleep(50);
            }

            // Store determined prop position
            if (visionPipeline.getPropPosition() != SidePosition.UNDEFINED) {
                // If position was found, do so
                propPosition = visionPipeline.getPropPosition();
                telemetry.addData("Found prop at position:", propPosition);
                telemetry.addData("Chroma average 1 (right):", visionPipeline.getChromaAverage1());
                telemetry.addData("Chroma average 2 (middle):", visionPipeline.getChromaAverage2());
                telemetry.addData("Chroma threshold:", Constants.VISION_CHROMA_THRESHOLD);
                telemetry.addData("Difference of chroma averages:", Math.abs(visionPipeline.getChromaAverage1() - visionPipeline.getChromaAverage2()));
                telemetry.update();
            } else {
                // If position was not found, fall back to the fallback prop position
                propPosition = Constants.FALLBACK_PROP_POSITION;
                telemetry.addData("Could not find prop.", "Falling back to fallback prop (" + propPosition + ")");
                telemetry.update();
            }

            // Pause the front camera's viewport to save CPU load
            frontCamera.pauseViewport();

        } else {
            // If the camera failed to open...
            // Fall back to the fallback prop position
            propPosition = Constants.FALLBACK_PROP_POSITION;
            telemetry.addData("Camera failed to open.", "Falling back to fallback prop (" + propPosition + ")");
            telemetry.update();
        }
        telemetry.addData("", "");
        telemetry.update();
    }

    /**
     * Turn to approximately face the prop, then drive to it on a linear path.
     */
    private void routineLinearToProp() {
        // TODO
    }

    /**
     * Drive and turn towards the center of the field on a linear path.
     */
    private void routineLinearToAway() {
        // TODO
    }

    /**
     * Drive and turn to approximately face the prop on a spline path.
     */
    private void routineSplineToProp() {
        // TODO
    }

    /**
     * Drive and turn towards the center of the field on a spline path.
     */
    private void routineSplineToAway() {
        // TODO
    }

    /*// Method to generate a pose by offsetting from the current pose
    private Pose2d offsetOfCurrentPose(double dx, double dy, double dh) {
        Pose2d estimate = mDrive.getPoseEstimate();
        return new Pose2d(estimate.getX() + dx, estimate.getY() + dy, estimate.getHeading() + dh);
    }

    // Method to generate a position by offsetting from the current position
    private Vector2d offsetOfCurrentPosition(double dx, double dy) {
        Pose2d estimate = mDrive.getPoseEstimate();
        return new Vector2d(estimate.getX() + dx, estimate.getY() + dy);
    }

    // Method to generate a heading by offsetting from the current heading
    private double offsetOfCurrentHeading(double dh) {
        Pose2d estimate = mDrive.getPoseEstimate();
        return estimate.getHeading() + dh;
    }*/
}
