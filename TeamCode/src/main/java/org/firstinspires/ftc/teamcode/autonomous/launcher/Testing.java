package org.firstinspires.ftc.teamcode.autonomous.launcher;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousRoutine;

@Autonomous(name="Autonomous Vision Tester", group="Autonomous Tester")
public class Testing extends AutonomousRoutine {
    @Override
    public void runOpMode() {
        selectedRoutine = Routine.TESTING;
        super.runOpMode();
    }
}
