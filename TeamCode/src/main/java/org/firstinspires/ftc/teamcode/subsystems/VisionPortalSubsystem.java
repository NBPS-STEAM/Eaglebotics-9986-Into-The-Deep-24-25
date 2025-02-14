package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants;
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
    public Pose3D getRobotPose() {
        return pickNearestPose(getDetections());
    }

    /**
     * Same as {@link #getRobotPose()}, but only with detections obtained after the last call to this method.
     * @see #getRobotPose()
     * @see #getFreshDetections()
     */
    public Pose3D getFreshRobotPose() {
        return pickNearestPose(getFreshDetections());
    }

    private Pose3D pickNearestPose(List<AprilTagDetection> detections) {
        if (detections == null) return null;
        double bestDist = Double.MAX_VALUE;
        Pose3D bestPose = null;
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && detection.ftcPose.range < bestDist) {
                bestDist = detection.ftcPose.range;
                bestPose = detection.robotPose;
            }
        }
        return bestDist < Constants.LOCALIZATION_VISION_RANGE ? bestPose : null;
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
}
