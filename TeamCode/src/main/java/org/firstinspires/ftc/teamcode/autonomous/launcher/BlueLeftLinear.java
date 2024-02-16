package org.firstinspires.ftc.teamcode.autonomous.launcher;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousRoutine;

/**
 * This is an example of a autonomous routine launcher with hardcoded parameters. For more
 * information on what exactly is going on here, see the explanation in LiveSelector.
 */
//@Disabled // If uncommented, this will prevent this opmode from appearing in the driver station.
@Autonomous(name="Linear Autonomous Routine (Blue-Left)", group="Autonomous Launcher")
public class BlueLeftLinear extends AutonomousRoutine {
    @Override
    public void runOpMode() {
        isBlueSide = true;
        isLongDistance = false;
        selectedRoutine = Routine.LINEAR;
        super.runOpMode();
    }
}
