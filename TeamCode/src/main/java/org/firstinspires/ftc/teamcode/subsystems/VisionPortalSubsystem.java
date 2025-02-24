package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.helper.Geo;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * A container class that groups together multiple subsystems that all utilize webcam(s).
 */
public class VisionPortalSubsystem extends SubsystemBase {
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    private ExposureControl exposureControl = null;
    private GainControl gainControl = null;

    private boolean exposureApplied = false;

    private GyroSource gyroSource = null;
    private Telemetry gyroTelemetry = null;

    public VisionPortalSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }

    public VisionPortalSubsystem(HardwareMap hardwareMap, boolean showDebug) {
        initAprilTag(hardwareMap, showDebug);
    }

    public void setProcessorEnabled(boolean enabled) {
        visionPortal.setProcessorEnabled(tagProcessor, enabled);
    }

    private void checkExposure() {
        if (!exposureApplied && isVisionPortalStreaming()) {
            setExposureMode(ExposureControl.Mode.Manual);
            setExposure(Constants.CAM_EXPOSURE_MS);
            exposureApplied = true;
        }
    }

    /**
     * @see AprilTagProcessor#getDetections()
     */
    public List<AprilTagDetection> getDetections() {
        checkExposure();
        return tagProcessor.getDetections();
    }

    /**
     * @see AprilTagProcessor#getFreshDetections()
     */
    public List<AprilTagDetection> getFreshDetections() {
        checkExposure();
        return tagProcessor.getFreshDetections();
    }

    /**
     * Estimates the robot's position and orientation on the field using the nearest AprilTag.
     * Perhaps it could be better to average the estimations from all visible AprilTags, but this could hurt precision
     * because distant AprilTags are less precise than near ones. Also, I don't want to deal with averaging angles.
     * @see #getFreshRobotPose()
     * @see #getDetections()
     */
    public Geo.Pose2D getRobotPose() {
        return pickNearestPose(getDetections());
    }

    /**
     * Same as {@link #getRobotPose()}, but only with detections obtained after the last call to this method.
     * @see #getRobotPose()
     * @see #getFreshDetections()
     */
    public Geo.Pose2D getFreshRobotPose() {
        return pickNearestPose(getFreshDetections());
    }

    private Geo.Pose2D pickNearestPose(List<AprilTagDetection> detections) {
        if (detections == null) return null;double bestDist = Double.MAX_VALUE;
        AprilTagDetection bestDet = null;
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && detection.ftcPose.range < bestDist) {
                bestDet = detection;
                bestDist = bestDet.ftcPose.range;
            }
        }
        if (bestDet == null) return null;
        if (bestDist > Constants.LOCALIZATION_VISION_RANGE) return null;
        if (gyroSource == null) {
            return pose3Dto2D(bestDet.robotPose);
        } else {
            return gyroPoseEstimator(bestDet);
        }
    }

    private Geo.Pose2D pose3Dto2D(Pose3D pose) {
        return new Geo.Pose2D(pose.getPosition().x, pose.getPosition().y, pose.getOrientation().getYaw(AngleUnit.RADIANS));
    }

    private Geo.Pose2D gyroPoseEstimator(AprilTagDetection detection) {
        // Introducing: Kelin's Botched Gyro-Vision Localizer!! (patent not pending)
        // This is the custom 2D localizer that can be used instead of the built-in one (AprilTagDetection.robotPose).

        // The goal of this pose estimator is to estimate the robot's field-relative pose, determining position using vision
        // and orientation using the robot's gyroscope instead of the orientation values obtained through vision.

        // Speaking from experience, the built-in pose estimator sometimes gives unreliable results for orientation (pitch, roll, yaw).
        // These results are then used to calculate the robotPose of the detection, which makes it equally unreliable.
        // Despite this, the results for relative XYZ and RBE (range, bearing, elevation) seem to be reliable.
        // Hence, this pose estimator, which uses the gyroscope for orientation instead of vision.
        // However, doing this to estimate a 3D pose would be hard. Let's simplify the process with some assumptions:
        // Because the robot (should) always be flat on the field and the camera (should) be mounted with no relative
        // roll/pitch to the AprilTags, we can assume that we won't have to worry about pitch (forward tilt),
        // roll (sideways tilt), or elevation. With these assumptions, the whole field can be simplified down to a 2D plane.
        // That's much more feasible!
        // This works so long as the robot doesn't tilt significantly (which shouldn't happen during auto driving anyway).

        // On paper, it might also be possible to implement a 3D localizer using the gyroscope, but that hasn't been done here.

        // Exit early if this tag can't be identified
        Geo.Vector2D tagPos = Constants.POS_APRIL_TAGS.get(detection.id);
        if (tagPos == null) return null;

        // First, offset the camera's distance from the AprilTag to find the robot's distance from the AprilTag relative to the camera's orientation
        /*Geo.Vector2D robotOffset = Geo.Vector2D.rotateBy(detection.ftcPose.x, detection.ftcPose.y, Constants.CAM_ORIENTATION_2D - (Math.PI / 2));

        double robotRX = robotOffset.x - Constants.CAM_POSITION_2D.y;
        double robotRY = robotOffset.y - Constants.CAM_POSITION_2D.x;

        // Then, calculate bearing and range from the robot's distance
        double robotBearing;
        if (isApproxZero(robotRX)) {
            // Edge case: robotRX approximately == 0
            robotBearing = robotRY > 0 ? Math.PI / 2 : -Math.PI / 2;
        } else if (isApproxZero(robotRY)) {
            // Edge case: robotRY approximately == 0
            robotBearing = robotRX > 0 ? 0 : Math.PI;
        } else {
            robotBearing = Math.atan(robotRY / robotRX);
            if (robotRX < 0) {
                if (robotRY > 0) {
                    robotBearing += Math.PI;
                } else {
                    robotBearing -= Math.PI;
                }
            }
        }
        double robotRange = Math.sqrt(robotRX*robotRX + robotRY*robotRY);

        // Then, estimate the robot's field-relative distance from the AprilTag using heading, bearing, and range
        double robotHeading = gyroSource.getFieldHeading();

        double robotDX = Math.cos(robotHeading + robotBearing) * robotRange;
        double robotDY = Math.sin(robotHeading + robotBearing) * robotRange;

        Geo.Vector2D camOffset = Geo.Vector2D.rotateBy(Constants.CAM_POSITION_2D.x, Constants.CAM_POSITION_2D.y, )
        robotDX +=

        // Add debug data if enabled
        if (gyroTelemetry != null) {
            gyroTelemetry.addData("robotOffset.x", robotOffset.x);
            gyroTelemetry.addData("robotOffset.y", robotOffset.y);
            gyroTelemetry.addData("robotRX", robotRX);
            gyroTelemetry.addData("robotRY", robotRY);
            gyroTelemetry.addData("robotBearing", robotBearing);
            gyroTelemetry.addData("robotRange", robotRange);
            gyroTelemetry.addData("robotHeading", robotHeading);
            gyroTelemetry.addData("robotDX", robotDX);
            gyroTelemetry.addData("robotDY", robotDY);
        }*/


        double heading = gyroSource.getFieldHeading();
        double bearing = heading + Math.toRadians(detection.ftcPose.bearing) + Constants.CAM_ORIENTATION_2D; // why is bearing reported in degrees?
        double range = detection.ftcPose.range;

        double cosB = Math.cos(bearing);
        double sinB = Math.sin(bearing);

        double dX = cosB * range;
        double dY = sinB * range;

        double oX = Constants.CAM_POSITION_2D.x * cosB - Constants.CAM_POSITION_2D.y * sinB;
        double oY = Constants.CAM_POSITION_2D.x * sinB + Constants.CAM_POSITION_2D.y * cosB;

        dX += oX;
        dY += oY;

        // Finally, offset from the robot's field-relative distance to find its field-relative position using the position of the AprilTag
        return new Geo.Pose2D(
                tagPos.x - dX,
                tagPos.y - dY,
                heading
        );
    }

    private boolean isApproxZero(double num) {
        return Math.abs(num) < 1e-7;
    }

    /**
     * Enables the custom gyro-vision localizer using the provided source for gyro readings (usually the drive subsystem).
     * <p>Pass null for gyroSource to disable the gyro-vision localizer.</p>
     */
    public void enableGyroLocalizer(GyroSource gyroSource) {
        enableGyroLocalizer(gyroSource, gyroTelemetry);
    }

    /** @see #enableGyroLocalizer(GyroSource)  */
    public void enableGyroLocalizer(GyroSource gyroSource, Telemetry debugTelemetry) {
        this.gyroSource = gyroSource;
        this.gyroTelemetry = debugTelemetry;
    }

    /**
     * Get the heading source being used for gyro-vision localization. If null, then gyro-vision localization is not in use.
     */
    public GyroSource getGyroLocalizer() {
        return gyroSource;
    }

    public boolean isVisionPortalStreaming() {
        return visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    public ExposureControl.Mode getExposureMode() {
        return getExposureControl().getMode();
    }

    public void setExposureMode(ExposureControl.Mode mode) {
        getExposureControl().setMode(mode);
    }

    public ExposureControl getExposureControl() {
        if (exposureControl == null && isVisionPortalStreaming()) exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        return exposureControl;
    }

    public void setExposure(long ms) {
        getExposureControl().setExposure(ms, TimeUnit.MILLISECONDS);
    }

    public long getExposure() {
        return getExposureControl().getExposure(TimeUnit.MILLISECONDS);
    }
    public long getMinExposure() {
        return getExposureControl().getMinExposure(TimeUnit.MILLISECONDS);
    }
    public long getMaxExposure() {
        return getExposureControl().getMaxExposure(TimeUnit.MILLISECONDS);
    }

    public GainControl getGainControl() {
        if (gainControl == null && isVisionPortalStreaming()) gainControl = visionPortal.getCameraControl(GainControl.class);
        return gainControl;
    }

    public void setGain(int gain) {
        getGainControl().setGain(gain);
    }

    public int getGain() {
        return getGainControl().getGain();
    }
    public int getMinGain() {
        return getGainControl().getMinGain();
    }
    public int getMaxGain() {
        return getGainControl().getMaxGain();
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag(HardwareMap hardwareMap, boolean showDebug) {

        // Create the AprilTag processor.
        tagProcessor = new AprilTagProcessor.Builder()
                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(Constants.CAM_POSITION, Constants.CAM_ORIENTATION)
                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(Constants.CAM_FX, Constants.CAM_FY, Constants.CAM_CX, Constants.CAM_CY)
                // ... these parameters are fx, fy, cx, cy.
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        tagProcessor.setDecimation(Constants.CAM_DECIMATION);

        // DEBUG
        if (Constants.estimator == 0) {
            tagProcessor.setPoseSolver(AprilTagProcessor.PoseSolver.APRILTAG_BUILTIN);
        } else if (Constants.estimator == 1) {
            tagProcessor.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_ITERATIVE);
        } else if (Constants.estimator == 2) {
            tagProcessor.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SOLVEPNP_EPNP);
        } else if (Constants.estimator == 3) {
            tagProcessor.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_IPPE);
        } else if (Constants.estimator == 4) {
            tagProcessor.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_IPPE_SQUARE);
        } else if (Constants.estimator == 5) {
            tagProcessor.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SQPNP);
        }
        // END DEBUG

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, Constants.NAME_CAMERA));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(Constants.CAM_SIZE_X, Constants.CAM_SIZE_Y));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(showDebug);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tagProcessor);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(tagProcessor, true);
    }


    public interface GyroSource {
        double getFieldHeading();
    }
}
