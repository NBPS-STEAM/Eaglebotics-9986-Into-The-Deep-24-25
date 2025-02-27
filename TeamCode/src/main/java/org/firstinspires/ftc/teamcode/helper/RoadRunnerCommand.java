package org.firstinspires.ftc.teamcode.helper;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.arcrobotics.ftclib.command.*;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystemRRVision;

import java.lang.Math;
import java.util.function.Supplier;

/**
 * Wraps a Road Runner {@link Action} in an FTCLib {@link Command} for use with the FTCLib command scheduler.
 * <p>This makes it possible to have autonomous driving in command-based opmodes!</p>
 * <p>There is also the option to retrieve the Action from a Supplier each time the command starts.
 * This is useful for reusing the same path multiple times or for paths that generate with runtime variables (i.e. starts at current pose).</p>
 */
public class RoadRunnerCommand extends CommandBase {
    private final TelemetryPacket packet = new TelemetryPacket(); // never actually used for anything but required by Action.run()
    private Supplier<Action> supplier = null;
    private Action action = null;
    protected boolean runAgain = true;

    public RoadRunnerCommand(Action action, Subsystem... requirements) {
        this.action = action;
        addRequirements(requirements);
    }

    public RoadRunnerCommand(Supplier<Action> supplier, Subsystem... requirements) {
        this.supplier = supplier;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        if (supplier != null) action = supplier.get();
        runAgain = true;
    }

    @Override
    public void execute() {
        if (runAgain) {
            runAgain = action.run(packet);
            packet.clearLines();
        }
    }

    @Override
    public boolean isFinished() {
        return !runAgain;
    }

    /**
     * Add an extra pose check at the end of the Action using vision.
     * <p>If an AprilTag is visible when the Action ends, it will wait a moment for a fresh detection
     * (max {@value FINAL_VISION_TIMEOUT} ms) before driving again to the final pose.</p>
     * <p>This can help get that extra bit of precision if a path ends right before a fresh detection.</p>
     */
    public CommandBase withVisionCheck(DriveSubsystemRRVision drive, Pose2d finalPose) {
        return new SequentialCommandGroup(
                this,
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(drive::didLastPoseEstUseVision).withTimeout(FINAL_VISION_TIMEOUT),
                                new RoadRunnerCommand(() -> genStrafeToLinearHeading(drive, finalPose, null, null))
                        ),
                        new InstantCommand(),
                        () -> hasDetections(drive)
                )
        );
    }
    
    public static final long FINAL_VISION_TIMEOUT = 80;


    // STATIC METHODS

    /**
     * Generate a command that will turn to a heading and immediately end at a certain distance threshold.
     * This can be used to avoid wasting time on path segments that don't need much precision.
     */
    public static CommandBase lazyTurnTo(DriveSubsystemRRVision drive, double heading, double threshold) {
        return new ParallelRaceGroup(
                new WaitUntilCommand(() -> withinRange(drive.pose.heading.toDouble(), heading, threshold)),
                new RoadRunnerCommand(() -> genTurnTo(drive, heading))
                // SuppliedRoadRunnerCommand is used because the initial pose needs to be calculated when the path starts.
        );
    }

    /**
     * Generate a command that will strafe to a position on the X axis and immediately end at a certain distance threshold.
     * This can be used to avoid wasting time on path segments that don't need much precision.
     */
    public static CommandBase lazyStrafeToXLinearHeading(DriveSubsystemRRVision drive, double posX, double heading, double threshold) {
        return new ParallelRaceGroup(
                new WaitUntilCommand(() -> withinRange(drive.pose.position.x, posX, threshold)),
                new RoadRunnerCommand(() -> genStrafeToXLinearHeading(drive, posX, heading))
                // SuppliedRoadRunnerCommand is used because the initial pose needs to be calculated when the path starts.
        );
    }

    public static CommandBase strafeToXLinearHeading(DriveSubsystemRRVision drive, double posX, double heading) {
        return new RoadRunnerCommand(() -> genStrafeToXLinearHeading(drive, posX, heading));
        // SuppliedRoadRunnerCommand is used because the initial pose needs to be calculated when the path starts.
    }

    /**
     * Generate a command that will strafe to a position and immediately end at a certain distance threshold.
     * This can be used to avoid wasting time on path segments that don't need much precision.
     */
    public static CommandBase lazyStrafeTo(DriveSubsystemRRVision drive, Vector2d pos, double threshold) {
        final double thresholdSqr = threshold * threshold;
        return new ParallelRaceGroup(
                new WaitUntilCommand(() -> withinRange(drive.pose.position, pos, thresholdSqr)),
                new RoadRunnerCommand(() -> genStrafeTo(drive, pos))
                // SuppliedRoadRunnerCommand is used because the initial pose needs to be calculated when the path starts.
        );
    }

    /**
     * Generate a command that will strafe to a position with linear heading and immediately end at a certain distance threshold.
     * This can be used to avoid wasting time on path segments that don't need much precision.
     * <p>Threshold is related to linear distance only. Heading is not considered in the distance calculation.</p>
     */
    public static CommandBase lazyStrafeToLinearHeading(DriveSubsystemRRVision drive, Pose2d pose, double threshold) {
        return lazyStrafeToLinearHeading(drive, pose, threshold, null, null);
    }

    /** @see #lazyStrafeToLinearHeading(DriveSubsystemRRVision, Pose2d, double)  */
    public static CommandBase lazyStrafeToLinearHeading(DriveSubsystemRRVision drive, Pose2d pose, double threshold, VelConstraint velOverride, AccelConstraint accelOverride) {
        final double thresholdSqr = threshold * threshold;
        return new ParallelRaceGroup(
                new WaitUntilCommand(() -> withinRange(drive.pose.position, pose.position, thresholdSqr)),
                new RoadRunnerCommand(() -> genStrafeToLinearHeading(drive, pose, velOverride, accelOverride))
                // SuppliedRoadRunnerCommand is used because the initial pose needs to be calculated when the path starts.
        );
    }

    public static CommandBase strafeToLinearHeading(DriveSubsystemRRVision drive, Pose2d pose) {
        return strafeToLinearHeading(drive, pose, null, null);
    }

    /** @see #strafeToLinearHeading(DriveSubsystemRRVision, Pose2d)  */
    public static CommandBase strafeToLinearHeading(DriveSubsystemRRVision drive, Pose2d pose, VelConstraint velOverride, AccelConstraint accelOverride) {
        return new RoadRunnerCommand(() -> genStrafeToLinearHeading(drive, pose, velOverride, accelOverride));
        // SuppliedRoadRunnerCommand is used because the initial pose needs to be calculated when the path starts.
    }

    private static boolean withinRange(double pos, double dest, double range) {
        return Math.abs(dest - pos) < range;
    }

    private static boolean withinRange(Vector2d pos, Vector2d dest, double rangeSqr) {
        double x = dest.x - pos.x;
        double y = dest.y - pos.y;
        return x * x + y * y < rangeSqr;
    }

    private static Action genTurnTo(DriveSubsystemRRVision drive, double heading) {
        return drive.actionBuilder(drive.pose)
                .turnTo(heading)
                .build();
    }

    private static Action genStrafeToXLinearHeading(DriveSubsystemRRVision drive, double posX, double heading) {
        return drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(posX, drive.pose.position.y), heading)
                .build();
    }

    private static Action genStrafeTo(DriveSubsystemRRVision drive, Vector2d pos) {
        return drive.actionBuilder(drive.pose)
                .strafeTo(pos)
                .build();
    }

    private static Action genStrafeToLinearHeading(DriveSubsystemRRVision drive, Pose2d pose, VelConstraint velOverride, AccelConstraint accelOverride) {
        return drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(pose.position, pose.heading, velOverride, accelOverride)
                .build();
    }
    
    private static boolean hasDetections(DriveSubsystemRRVision drive) {
        return drive.vps != null && drive.vps.hasDetections();
    }
}
