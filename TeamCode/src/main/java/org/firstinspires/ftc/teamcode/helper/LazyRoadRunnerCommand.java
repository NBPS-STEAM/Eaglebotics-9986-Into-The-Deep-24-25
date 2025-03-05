package org.firstinspires.ftc.teamcode.helper;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.arcrobotics.ftclib.command.*;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystemRRVision;

import java.lang.Math;
import java.util.function.Supplier;

/**
 * Generate a command that will wrap a Road Runner Action and immediately end at a certain distance threshold.
 * This can be used to avoid wasting time on path segments that don't need much precision.
 * <p>Wraps a Road Runner {@link Action} in an FTCLib {@link Command} for use with the FTCLib command scheduler.</p>
 * <p>This makes it possible to have autonomous driving in command-based opmodes!</p>
 * <p>There is also the option to retrieve the Action from a Supplier each time the command starts.
 * This is useful for reusing the same path multiple times or for paths that generate with runtime variables (i.e. starts at current pose).</p>
 */
public class LazyRoadRunnerCommand extends CommandBase {
    /*private final TelemetryPacket packet = new TelemetryPacket(); // never actually used for anything but required by Action.run()
    private Supplier<Action> supplier = null;
    private Action action = null;
    private final Vector2d finalPos;
    private final Double finalHeading;
    private final double threshold;
    private final double thresholdSqr;
    protected boolean runAgain = true;


    public LazyRoadRunnerCommand(Supplier<Action> supplier, Vector2d finalPos, double threshold, Subsystem... requirements) {
        this.supplier = supplier;
        this.finalPos = finalPos;
        this.finalHeading = null;
        this.threshold = threshold;
        this.thresholdSqr = threshold*threshold;
        addRequirements(requirements);
    }

    public LazyRoadRunnerCommand(Supplier<Action> supplier, double finalHeading, double threshold, Subsystem... requirements) {
        this.supplier = supplier;
        this.finalPos = null;
        this.finalHeading = finalHeading;
        this.threshold = threshold;
        this.thresholdSqr = threshold*threshold;
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
        if (finalPos == null) {
            if (withinRange(drive))
        } else {

        }

    }

    @Override
    public boolean isFinished() {
        return !runAgain;
    }


    // STATIC METHODS

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
    }*/
}
