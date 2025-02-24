package org.firstinspires.ftc.teamcode.helper.localization;

import com.acmerobotics.roadrunner.*;
import org.firstinspires.ftc.teamcode.helper.Geo;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystemRRVision;
import org.firstinspires.ftc.teamcode.subsystems.VisionPortalSubsystem;

import java.lang.Math;

/**
 * A localizer that makes use of vision to localize using AprilTags.
 * Falls back to a regular encoder-based DriveLocalizer when vision isn't available.
 * <p>Requires an alliance to be set!</p>
 */
public class VisionDriveLocalizer implements Localizer {

    public final DriveSubsystemRRVision drive;
    public final DriveLocalizer encoderLocalizer;
    public final VisionPortalSubsystem vps;

    private boolean isBlueAlliance = false;

    private Pose2d absolutePose = null;
    private Localizers.Methods lastUpdateMethod = Localizers.Methods.UNSPECIFIED;

    /** Remember to set an alliance with {@link #setAlliance(boolean)} before use! */
    public VisionDriveLocalizer(DriveSubsystemRRVision drive, VisionPortalSubsystem visionPortalSubsystem) {
        this.drive = drive;
        this.encoderLocalizer = new DriveLocalizer(drive);
        this.vps = visionPortalSubsystem;
    }

    @Override
    public Twist2dDual<Time> update() {
        Twist2dDual<Time> encoderTwist = encoderLocalizer.update();
        Pose2d visionPose = getVisionPose();

        // Record absolute pose and whether it could be obtained using vision
        absolutePose = visionPose;
        lastUpdateMethod = visionPose == null ? Localizers.Methods.ENCODERS : Localizers.Methods.VISION;

        // We always want to use encoders for change in position/velocity because they're more precise, so return that
        return encoderTwist;
    }

    @Override
    public Pose2d getAbsolutePosition() {
        return absolutePose;
    }

    private Pose2d getVisionPose() {
        return robotPoseAsAlliance(vps.getFreshRobotPose());
    }

    private Pose2d robotPoseAsAlliance(Geo.Pose2D robotPose) {
        if (robotPose == null) return null;
        if (isBlueAlliance) {
            return new Pose2d(
                    -robotPose.x,
                    -robotPose.y,
                    robotPose.heading + Math.PI
            );
        } else {
            return new Pose2d(
                    robotPose.x,
                    robotPose.y,
                    robotPose.heading
            );
        }
    }

    @Override
    public Localizers.Methods getLastUpdateMethod() {
        return lastUpdateMethod;
    }

    @Override
    public void setAlliance(boolean isBlueAlliance) {
        this.isBlueAlliance = isBlueAlliance;
    }
}
