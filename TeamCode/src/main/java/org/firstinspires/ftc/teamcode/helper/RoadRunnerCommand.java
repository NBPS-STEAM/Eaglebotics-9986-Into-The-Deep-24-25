package org.firstinspires.ftc.teamcode.helper;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;

import java.util.Set;

/**
 * Wraps a Road Runner {@link com.acmerobotics.roadrunner.Action Action} in an FTCLib
 * {@link com.arcrobotics.ftclib.command.Command Command} for use with the FTCLib command scheduler.
 * <p>This makes it possible to have autonomous driving in command-based opmodes!</p>
 */
public class RoadRunnerCommand implements Command {
    private final TelemetryPacket packet = new TelemetryPacket();
    private final Set<Subsystem> requirements;
    private final Action action;
    private boolean runAgain = true;

    public RoadRunnerCommand(Action action, Subsystem... requirements) {
        this.action = action;
        this.requirements = Set.of(requirements);
    }

    @Override
    public void execute() {
        if (runAgain) {
            runAgain = action.run(packet);
        }
    }

    @Override
    public boolean isFinished() {
        return !runAgain;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }
}
