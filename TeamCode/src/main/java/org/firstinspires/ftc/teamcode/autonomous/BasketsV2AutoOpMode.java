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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.helper.IntakeState;
import org.firstinspires.ftc.teamcode.helper.DriverPrompter;
import org.firstinspires.ftc.teamcode.helper.ResetZeroState;
import org.firstinspires.ftc.teamcode.helper.RoadRunnerCommand;
import org.firstinspires.ftc.teamcode.helper.localization.Localizers;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystemRRVision;
import org.firstinspires.ftc.teamcode.subsystems.VisionPortalSubsystem;
import org.firstinspires.ftc.teamcode.teleop.MecautoTeleOpMode;
import org.jetbrains.annotations.NotNull;

import java.lang.Math;

/*
 * This file contains a simple example "OpMode" for driving a robot.
 * An OpMode (Operation Mode) is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The OpMode to use can be selected on the FTC Driver Station.
 *
 * Hardware names are listed in the Constants file. You must correctly set all hardware names for each
 * motor/servo/etc. in the Driver Station for the code to find your devices.
 *
 * This code goes from the starting position (second tile, ~40 in. from the left wall and facing away from the back wall),
 * scores a preloaded sample in the high basket, collects and scores a few spike mark samples in the high basket,
 * and then goes to park in the ascent zone (ascent level 1).
 * This version has some optimizations to go a little faster and score all four samples.
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

@Autonomous(name="Baskets V2 Auto-OpMode (3 or 4 samples)", group="Autonomous OpMode")
public class BasketsV2AutoOpMode extends CommandOpMode {

    // Variables
    private static final Pose2d INITIAL_POSE = new Pose2d(-35, -60, Math.PI / 2);
    private static final long OUTTAKE_DELAY_MS = 400;

    // Hardware Variables
    private DriveSubsystemRRVision drive;
    private ArmSubsystem armSubsystem;
    private VisionPortalSubsystem visionPortalSubsystem;


    // This is run when the "INIT" button is pressed on the Driver Station.
    @Override
    public void initialize() {
        // Reset zero state; autonomous opmodes should always zero the robot
        ResetZeroState.resetZeroState();
        // Initialize hardware
        armSubsystem = new ArmSubsystem(hardwareMap, 0.5, 0.5, 1.0);
        visionPortalSubsystem = new VisionPortalSubsystem(hardwareMap, Constants.CAM_DO_STREAM);

        // Initialize MecanumDrive at a particular pose
        drive = new DriveSubsystemRRVision(hardwareMap, visionPortalSubsystem,
                Localizers.ENCODERS_WITH_VISION, INITIAL_POSE);

        // Autonomous path is generated and scheduled at the bottom of this method.


        // Schedule commands

        // When the intake intakes a sample, stop the intake
        new Trigger(armSubsystem::shouldStopIntakeForSample).whenActive(armSubsystem::stopIntake);

        // At all times, update the telemetry log
        drive.new TelemetryLoggerCommand(telemetry).schedule(false);


        // Get driver configuration
        // This must be done last, as it waits for user input (which could take until the end of init)

        // Wait until camera is ready (this will make it obvious if it doesn't activate)
        MecautoTeleOpMode.sleepForVisionPortal(this, visionPortalSubsystem, telemetry);

        // Alliance
        drive.setIsBlueAlliance(DriverPrompter.queryAlliance(this), DriverPrompter.wasAllianceFromDriver());

        // Do left sample?
        // Path is generated and scheduled here.
        Command pathCommand = getBasketsV2(drive, armSubsystem,
                DriverPrompter.queryBoolean(this, true, "Go for the leftmost sample?", "Do left sample"));
        pathCommand.andThen(new InstantCommand(this::reportFinished)).schedule(false);

        telemetry.addLine("Baskets V2 Auto Ready!");
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

    private Command getBasketsV2(DriveSubsystemRRVision drive, ArmSubsystem armSubsystem, boolean doLeftSample) {
        // Define keypoint positions
        final Pose2d BASKET_POSE = Constants.POS_BASKETS_SCORE;
        final Pose2d SAMPLE_1_APP = new Pose2d(-49, -43, Math.toRadians(92.35));
        final Pose2d SAMPLE_1_INT = new Pose2d(-49, -37, Math.toRadians(92.35));
        final Pose2d SAMPLE_2_APP = new Pose2d(-59, -43, Math.toRadians(92.35));
        final Pose2d SAMPLE_2_INT = new Pose2d(-59, -37, Math.toRadians(92.35));
        final Pose2d SAMPLE_3_APP1 = new Pose2d(-57, -45.5, Math.toRadians(100));
        final Pose2d SAMPLE_3_APP2 = new Pose2d(-59, -45, Math.toRadians(117.4));
        final Pose2d SAMPLE_3_INT = new Pose2d(-59, -42.5, Math.toRadians(117.4));
        final Pose2d PARK_1 = new Pose2d(-51, -12, 0);
        final Pose2d PARK_2 = new Pose2d(-30, -12, 0);

        // Compose some commands
        Command preloadScore = composeScoreCommand(BASKET_POSE);

        Command park = new RoadRunnerCommand(drive.actionBuilder(BASKET_POSE)
                // Park in ascent zone and achieve Level 1 Ascent
                .afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.STOPPED))
                .stopAndAdd(blockIfTimeout(2.0))
                .afterTime(0.5, () -> armSubsystem.applyNamedPosition("ascent level 1"))
                .strafeToLinearHeading(PARK_1.position, PARK_1.heading)
                .strafeToLinearHeading(PARK_2.position, PARK_2.heading)
                .afterTime(0.0, () -> armSubsystem.setRotationPower(0.0))
                .build());


        // Compose final command
        if (doLeftSample) {
            return new SequentialCommandGroup(
                    preloadScore,
                    composeSampleCommand(BASKET_POSE, SAMPLE_1_APP, SAMPLE_1_INT),
                    composeSampleCommand(BASKET_POSE, SAMPLE_2_APP, SAMPLE_2_INT),
                    composeSampleSafeCommand(BASKET_POSE, SAMPLE_3_APP1, SAMPLE_3_APP2, SAMPLE_3_INT),
                    park
            );
        } else {
            return new SequentialCommandGroup(
                    preloadScore,
                    composeSampleCommand(BASKET_POSE, SAMPLE_1_APP, SAMPLE_1_INT),
                    composeSampleCommand(BASKET_POSE, SAMPLE_2_APP, SAMPLE_2_INT),
                    park
            );
        }
    }

    private Command composeSampleCommand(Pose2d BASKET_POSE, Pose2d APPROACH_POSE, Pose2d INTAKE_POSE) {
        Command approach = new RoadRunnerCommand(drive.actionBuilder(BASKET_POSE)
                // Approach spike mark sample
                .afterTime(0.5, () -> armSubsystem.applyNamedPosition("intake ground-high"))
                .strafeToLinearHeading(APPROACH_POSE.position, APPROACH_POSE.heading)
                .build());

        Command intake = new RoadRunnerCommand(drive.actionBuilder(APPROACH_POSE)
                // Intake spike mark sample
                .strafeToLinearHeading(INTAKE_POSE.position, INTAKE_POSE.heading)
                .build());

        Command score = composeScoreCommand(BASKET_POSE);

        Command invalidateTargets = new InstantCommand(armSubsystem::invalidateTargets);
        Command yieldArmTarget = new WaitUntilCommand(() -> armSubsystem.isRotationAtTargetPosition(55));
        Command yieldIntakeSensor = new WaitUntilCommand(armSubsystem::hasSampleInIntake);

        return new SequentialCommandGroup(
                invalidateTargets, // to prevent yieldArmTarget from ending before the arm position is set
                new ParallelDeadlineGroup(
                        yieldArmTarget,
                        approach
                ),
                new ParallelRaceGroup(
                        yieldIntakeSensor,
                        intake
                ),
                score
        );
    }

    private Command composeSampleSafeCommand(Pose2d BASKET_POSE, Pose2d APPROACH_POSE_1, Pose2d APPROACH_POSE_2, Pose2d INTAKE_POSE) {
        Command approach1 = new RoadRunnerCommand(drive.actionBuilder(BASKET_POSE)
                // Approach spike mark sample (first part)
                .afterTime(0.5, () -> armSubsystem.applyNamedPosition("intake ground-high"))
                .strafeToLinearHeading(APPROACH_POSE_1.position, APPROACH_POSE_1.heading)
                .build());

        Command approach2 = new RoadRunnerCommand(() -> drive.actionBuilder(drive.pose)
                // Approach spike mark sample (second part)
                .strafeToLinearHeading(APPROACH_POSE_2.position, APPROACH_POSE_2.heading)
                .build());

        Command intake = new RoadRunnerCommand(drive.actionBuilder(APPROACH_POSE_2)
                // Intake spike mark sample
                .strafeToLinearHeading(INTAKE_POSE.position, INTAKE_POSE.heading)
                .build());

        Command score = composeScoreCommand(BASKET_POSE);

        Command invalidateTargets = new InstantCommand(armSubsystem::invalidateTargets);
        Command yieldArmSafe = new WaitUntilCommand(() -> armSubsystem.getRotationPosition() < 0.4);
        Command yieldArmTarget = new WaitUntilCommand(() -> armSubsystem.isRotationAtTargetPosition(55));
        Command yieldIntakeSensor = new WaitUntilCommand(armSubsystem::hasSampleInIntake);

        return new SequentialCommandGroup(
                invalidateTargets, // to prevent yieldArmTarget from ending before the arm position is set
                new ParallelDeadlineGroup(
                        yieldArmTarget,
                        new SequentialCommandGroup(
                                new ParallelDeadlineGroup(
                                        yieldArmSafe,
                                        approach1
                                ),
                                approach2
                        )
                ),
                new ParallelRaceGroup(
                        yieldIntakeSensor,
                        intake
                ),
                score
        );
    }

    private Command composeScoreCommand(Pose2d BASKET_POSE) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> armSubsystem.applyIntakeState(IntakeState.STOPPED)),
                new InstantCommand(() -> armSubsystem.applyNamedPosition("basket high")),
                new RoadRunnerCommand(() -> drive.actionBuilder(drive.pose)
                        .strafeTo(BASKET_POSE.position)
                        .stopAndAdd(armSubsystem.yieldForArmTarget())
                        .turnTo(BASKET_POSE.heading)
                        .build()).withVisionCheck(drive, BASKET_POSE),
                new InstantCommand(() -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE)),
                new WaitCommand(OUTTAKE_DELAY_MS)
        );
        // This command starts with FTCLib Commands, then switches to Road Runner Actions for a bit, then goes back to FTCLib.
        // A command that casually flips between two different command systems. That's fun.
    }

    /**
     * Constructs an Action that runs indefinitely if there isn't enough time remaining in the auto period.
     * Optionally takes a Runnable to run if/when it starts blocking.
     */
    private Action blockIfTimeout(double minTime) {
        return blockIfTimeout(minTime, null);
    }

    /**
     * @see #blockIfTimeout(double)
     */
    private Action blockIfTimeout(double minTime, Runnable whenBlock) {
        return new BlockIfTimeout(minTime, whenBlock);
    }

    class BlockIfTimeout implements Action {
        private final double minTime;
        private final Runnable whenBlock;
        private boolean startedBlocking = false;

        public BlockIfTimeout(double minTime, Runnable whenBlock) {
            this.minTime = minTime;
            this.whenBlock = whenBlock;
        }

        @Override
        public boolean run(@NotNull TelemetryPacket telemetryPacket) {
            boolean cond = 30 - getRuntime() < minTime;
            if (cond && !startedBlocking && whenBlock != null) {
                whenBlock.run();
            }
            startedBlocking = cond;
            return cond; // If true, this action will run again
        }
    }
}
