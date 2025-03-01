/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.helper.DriverPrompter;
import org.firstinspires.ftc.teamcode.helper.IntakeState;
import org.firstinspires.ftc.teamcode.helper.ResetZeroState;
import org.firstinspires.ftc.teamcode.helper.RoadRunnerCommand;
import org.firstinspires.ftc.teamcode.helper.localization.Localizers;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystemRRVision;
import org.firstinspires.ftc.teamcode.subsystems.VisionPortalSubsystem;
import org.firstinspires.ftc.teamcode.teleop.MecautoTeleOpMode;
import org.jetbrains.annotations.NotNull;

import java.lang.Math;
import java.util.Arrays;
import java.util.function.Supplier;

/*
 * This file contains a simple example "OpMode" for driving a robot.
 * An OpMode (Operation Mode) is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The OpMode to use can be selected on the FTC Driver Station.
 *
 * Hardware names are listed in the Constants file. You must correctly set all hardware names for each
 * motor/servo/etc. in the Driver Station for the code to find your devices.
 *
 * This code goes from the starting position (third tile, ~60 in. from the right wall and facing away from the back wall),
 * scores a preloaded specimen on the high bar, collects and scores a second specimen from the observation zone on the
 * high bar, and then goes to park in the observation zone.
 *
 * Before starting this OpMode, the arm lift must be in its lowest position and the extension fully
 * retracted. The robot must also be facing forward.
 *
 * See MecanumTeleOpMode for more basic info.
 *
 *
 * The initialize() method runs when the "init" button is pressed on the Driver Station.
 * Everything afterward is handled by commands.
 *
 *
 * Tips:
 * If you don't understand what a particular method does, then hover your mouse over it to get some info and a short description.
 * If you click on a variable/method, then all uses of it in the file will be highlighted.
 * Click on a variable/method and press Ctrl+B (Cmd+B on mac) to be taken to where it was originally declared.
 * In addition, do that twice for a list of everywhere that variable/method is used.
 * You can also do that by holding ctrl while you click on the variable/method.
 */

@Autonomous(name="Specimens V1 Auto-OpMode (2-4 specimens)", group="Autonomous OpMode")
public class SpecimensV1AutoOpMode extends CommandOpMode {

    // Variables
    private static final Pose2d INITIAL_POSE = new Pose2d(7, -63.5, Math.PI / 2);
    private static final double INTAKE_X = 48;
    private static final double INTAKE_Y = -50;
    private static final double INTAKE_HEADING = Math.toRadians(4);
    private static final double APPROACH_Y = -48;
    private static final double SCORE_Y = -35;

    // Hardware Variables
    private DriveSubsystemRRVision drive;
    private ArmSubsystem armSubsystem;
    private VisionPortalSubsystem visionPortalSubsystem;


    // This is run when the "INIT" button is pressed on the Driver Station.
    @Override
    public void initialize() {
        // Reset zero state; autonomous opmodes should always zero the robot
        ResetZeroState.resetZeroState();

        // Do which routine?
        // Path is NOT generated here.
        // This is asked all the way up here because drive subsystem params depend on this.
        boolean useV2 = DriverPrompter.queryBoolean(this, false, "Do the full 4-specimen routine?", "Do full routine");
        Supplier<Command> pathGenerator;
        DriveSubsystemRRVision.Params params = new DriveSubsystemRRVision.Params();
        if (useV2) {
            pathGenerator = this::getSpecimensV2;
            params.maxWheelVel = 50;
            params.minProfileAccel = -20;
            params.maxProfileAccel = 20.0;
            params.maxAngVel = Math.PI * 1.5;
            params.maxAngAccel = Math.PI * 1.5;
        } else {
            pathGenerator = this::getSpecimensV1;
        }

        // Initialize hardware
        armSubsystem = new ArmSubsystem(hardwareMap, 0.5, 0.8, 1.0);
        visionPortalSubsystem = new VisionPortalSubsystem(hardwareMap, Constants.CAM_DO_STREAM);

        // Initialize MecanumDrive at a particular pose
        drive = new DriveSubsystemRRVision(hardwareMap, visionPortalSubsystem,
                Localizers.ENCODERS_WITH_VISION, INITIAL_POSE, params);

        // Generate path
        Command pathCommand = pathGenerator.get().andThen(new InstantCommand(this::reportFinished));


        // Schedule commands

        // When this opmode starts, run the path
        pathCommand.schedule(false);

        // When the intake intakes a sample, stop the intake
        new Trigger(armSubsystem::shouldStopIntakeForSample).whenActive(armSubsystem::stopIntake);

        // At all times, update the telemetry log
        drive.new TelemetryLoggerCommand(telemetry).schedule(false);
        telemetry.addData("Runtime", this::getRuntime);


        // Get driver configuration
        // This must be done last, as it waits for user input (which could take until the end of init)

        // Wait until camera is ready (this will make it obvious if it doesn't activate)
        MecautoTeleOpMode.sleepForVisionPortal(this, visionPortalSubsystem, telemetry);

        // Alliance
        drive.setIsBlueAlliance(DriverPrompter.queryAlliance(this), DriverPrompter.wasAllianceFromDriver());

        telemetry.addLine("Specimens V1 Auto Ready!");
        telemetry.update();
    }


    // This runs when the routine ends or is stopped. Look inside the CommandOpMode class to see how it works.
    // Don't forget to call super.reset() to properly shutdown the opmode!
    @Override
    public void reset() {
        ResetZeroState.markToNotZeroOnInit(drive.pose);
        super.reset();
    }


    private void reportFinished() {
        telemetry.addData("Finished! Time", getRuntime()).setRetained(true);
        telemetry.update();
    }

    private Command getSpecimensV1() {
        return new SequentialCommandGroup(
                composeScoreCommand(8),
                composeIntakeCommand(),
                composeScoreCommand(8), //TODO: MIGHT NOT WORK (CHANGE TO 0 IF NOT)
                composeRetrievalCommand()
        );
    }

    private Command getSpecimensV2() {
        return new SequentialCommandGroup(
                composeScoreCommand(8),
                composeRetrievalCommand(),
                RoadRunnerCommand.lazyStrafeTo(drive, new Vector2d(-9, INTAKE_Y+4), 6),
                composeIntakeCommand(),
                composeScoreCommand(8),
                composeIntakeCommand(),
                composeScoreCommand(8),
                composeIntakeCommand(),
                composeScoreCommand(8),
                composeIntakeCommand(),
                composeScoreCommand(8),
                composeParkCommand()
        );
    }

    private Command composeIntakeCommand() {
        return new SequentialCommandGroup(
                new RoadRunnerCommand(() -> drive.actionBuilder(drive.pose).turnTo(INTAKE_HEADING).build()),
                new InstantCommand(() -> armSubsystem.applyNamedPosition("intake ground")),
                new ParallelRaceGroup(
                        RoadRunnerCommand.lazyStrafeToLinearHeading(drive, new Pose2d(INTAKE_X, INTAKE_Y, INTAKE_HEADING), 5),
                        new WaitUntilCommand(armSubsystem::hasSampleInIntake)
                )
        );
    }

    private Command composeScoreCommand(double alignX) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> armSubsystem.applyIntakeState(IntakeState.STOPPED)),
                new InstantCommand(() -> armSubsystem.applyNamedPosition("specimen low")),
                RoadRunnerCommand.strafeToXLinearHeading(drive, alignX, Math.PI / 2),
                new ParallelDeadlineGroup(
                        RoadRunnerCommand.lazyStrafeToLinearHeading(drive, new Pose2d(alignX, SCORE_Y, Math.PI / 2), 10),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> drive.pose.position.y > APPROACH_Y),
                                new InstantCommand(() -> armSubsystem.applyNamedPosition("specimen high"))
                        )
                ),
                new InstantCommand(() -> armSubsystem.applyExtensionPosition(0)),
                RoadRunnerCommand.strafeToLinearHeading(drive, new Pose2d(-3, INTAKE_Y+4, Math.PI / 2))
        );
    }

    private Command composeParkCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> armSubsystem.applyIntakeState(IntakeState.STOPPED)),
                new InstantCommand(() -> armSubsystem.applyNamedPosition("stow")),
                new RoadRunnerCommand(() -> drive.actionBuilder(drive.pose).strafeTo(new Vector2d(INTAKE_X+12, INTAKE_Y-6)).build())
        );
    }

    private Command composeRetrievalCommand() {
        // Define keypoint positions
        final double retYFar = -16;
        final double retYNear = -60;
        final double retX1 = 38;
        final double retX2 = 47;
        final double retX3 = 56;
        //final double retX4 = 58;

        final VelConstraint fastVel =
                new MinVelConstraint(Arrays.asList(
                        drive.kinematics.new WheelVelConstraint(100),
                        new AngularVelConstraint(Math.PI)
                ));
        final AccelConstraint fastAccel =
                new ProfileAccelConstraint(-30, 30);

        // Compose sample retrieval command
        return new SequentialCommandGroup(
                new InstantCommand(() -> armSubsystem.applyNamedPosition("compact")),
                RoadRunnerCommand.lazyStrafeToLinearHeading(drive, forwardPose(retX1, -48), 2, fastVel, fastAccel),
                RoadRunnerCommand.lazyStrafeToLinearHeading(drive, forwardPose(retX1, retYFar+20), 20, fastVel, fastAccel),

                RoadRunnerCommand.lazyStrafeToLinearHeading(drive, forwardPose(retX2, retYFar), 5, fastVel, fastAccel),
                RoadRunnerCommand.lazyStrafeToLinearHeading(drive, forwardPose(retX2, retYNear), 20, fastVel, fastAccel),
                RoadRunnerCommand.lazyStrafeToLinearHeading(drive, forwardPose(retX2, retYFar+20), 20, fastVel, fastAccel),

                RoadRunnerCommand.lazyStrafeToLinearHeading(drive, forwardPose(retX3, retYFar), 5, fastVel, fastAccel),
                RoadRunnerCommand.lazyStrafeToLinearHeading(drive, forwardPose(retX3, retYNear), 20, fastVel, fastAccel)
                //RoadRunnerCommand.lazyStrafeToLinearHeading(drive, forwardPose(retX3, retYFar+20), 20, fastVel, fastAccel),

                //RoadRunnerCommand.strafeToLinearHeading(drive, new Pose2d(retX3, retYFar, Math.PI * 3 / 8), fastVel, fastAccel),
                //RoadRunnerCommand.lazyStrafeToLinearHeading(drive, new Pose2d(retX3, retYNear, Math.PI * 3 / 8), 20, fastVel, fastAccel)
        );
    }

    private Pose2d forwardPose(double x, double y) {
        return new Pose2d(x, y, Math.PI / 2);
    }
}
