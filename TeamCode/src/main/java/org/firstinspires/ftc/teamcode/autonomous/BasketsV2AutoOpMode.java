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

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.arcrobotics.ftclib.command.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.helper.*;
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

@Autonomous(name="Baskets V2 Auto-OpMode (3 samples)", group="Autonomous OpMode")
@Config
public class BasketsV2AutoOpMode extends CommandOpMode {

    // Variables
    private static final Pose2d INITIAL_POSE = new Pose2d(-35, -60, Math.PI / 2);
    private static final long BASKET_DELAY_MS = 800;
    private static final long OUTTAKE_DELAY_MS = 400;

    public static double debugMaxAngVel = Math.PI * 0.6;
    public static double debugMaxAngAccel = Math.PI * 0.6;
    public static double debugMaxWheelVel = 40;
    public static double debugMaxProfileAccel = 20;
    public static double debugMinProfileAccel = -20;

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
        armSubsystem = new ArmSubsystem(hardwareMap, 0.5, 0.6, 1.0);
        visionPortalSubsystem = new VisionPortalSubsystem(hardwareMap, Constants.CAM_DO_STREAM);

        // Initialize MecanumDrive at a particular pose
        DriveSubsystemRRVision.Params params = new DriveSubsystemRRVision.Params();
        params.maxAngVel = debugMaxAngVel;
        params.maxAngAccel = debugMaxAngAccel;
        params.maxWheelVel = debugMaxWheelVel;
        params.maxProfileAccel = debugMaxProfileAccel;
        params.minProfileAccel = debugMinProfileAccel;
        drive = new DriveSubsystemRRVision(hardwareMap, visionPortalSubsystem,
                Localizers.ENCODERS_WITH_VISION, INITIAL_POSE, params);
        visionPortalSubsystem.maxRange = 40;

        // Autonomous path is generated and scheduled at the bottom of this method.


        // Schedule commands

        // When the intake intakes a sample, stop the intake
        //new Trigger(armSubsystem::shouldStopIntakeForSample).whenActive(armSubsystem::stopIntake);

        // At all times, update the telemetry log
        drive.new TelemetryLoggerCommand(telemetry).schedule(false);
        telemetry.addData("Runtime", this::getRuntime);
        telemetry.addData("Sample in Intake?", armSubsystem::hasSampleInIntake);


        // Get driver configuration
        // This must be done last, as it waits for user input (which could take until the end of init)

        // Wait until camera is ready (this will make it obvious if it doesn't activate)
        MecautoTeleOpMode.sleepForVisionPortal(this, visionPortalSubsystem, telemetry);

        // Alliance
        drive.setIsBlueAlliance(DriverPrompter.queryAlliance(this), DriverPrompter.wasAllianceFromDriver());

        // Do left sample?
        // Path is generated and scheduled here.
        //Command pathCommand = getBasketsV2(DriverPrompter.queryBoolean(this, true, "Go for the leftmost sample?", "Do left sample"));
        //pathCommand.andThen(new InstantCommand(this::reportFinished)).schedule(false);
        getBasketsV2(false).schedule(false);

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

    private Command getBasketsV2(boolean doLeftSample) {
        // Define keypoint positions
        final Pose2d BASKET_POSE = Constants.POS_BASKETS_SCORE;
        final Pose2d BASKET_POSE_FIRST = new Pose2d(new Vector2d(BASKET_POSE.position.x - 0.25, BASKET_POSE.position.y - 0.25), BASKET_POSE.heading);
        final Pose2d SAMPLE_1_APP = new Pose2d(-47, -49, Math.toRadians(92.35));
        final Pose2d SAMPLE_1_INT = new Pose2d(-47, -41, Math.toRadians(92.35));
        final Pose2d SAMPLE_2_APP = new Pose2d(-59, -49, Math.toRadians(92.35));
        final Pose2d SAMPLE_2_INT = new Pose2d(-59, -41, Math.toRadians(92.35));
        //final Pose2d SAMPLE_2_SCAN = new Pose2d(new Vector2d(BASKET_POSE.position.x + 20, BASKET_POSE.position.y), -Math.PI / 2);
        final Pose2d SAMPLE_3_APP1 = new Pose2d(-57, -45.5, Math.toRadians(100));
        final Pose2d SAMPLE_3_APP2 = new Pose2d(-60, -45, Math.toRadians(117.4));
        final Pose2d SAMPLE_3_INT = new Pose2d(-60, -44, Math.toRadians(117.4));
        final Pose2d PARK_1 = new Pose2d(-51, -8, 0);
        final Pose2d PARK_2 = new Pose2d(-28, -12, 0);

        // DEBUG
        /*return new SequentialCommandGroupFix(
                composeScoreCommandArmless(BASKET_POSE),
                new WaitCommand(1000),
                new RoadRunnerCommand(() -> drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(SAMPLE_1_APP.position, SAMPLE_1_APP.heading)
                        .strafeToLinearHeading(SAMPLE_1_INT.position, SAMPLE_1_INT.heading)
                        .build()),
                composeScoreCommandArmless(BASKET_POSE),
                new WaitCommand(1000),
                new RoadRunnerCommand(() -> drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(SAMPLE_2_APP.position, SAMPLE_2_APP.heading)
                        .strafeToLinearHeading(SAMPLE_2_INT.position, SAMPLE_2_INT.heading)
                        .build()),
                composeScoreCommandArmless(BASKET_POSE),
                new WaitCommand(1000),
                new RoadRunnerCommand(() -> drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(SAMPLE_3_APP1.position, SAMPLE_3_APP1.heading)
                        .strafeToLinearHeading(SAMPLE_3_APP2.position, SAMPLE_3_APP2.heading)
                        .strafeToLinearHeading(SAMPLE_3_INT.position, SAMPLE_3_INT.heading)
                        .build()),
                composeScoreCommandArmless(BASKET_POSE),
                new WaitCommand(1000),
                RoadRunnerCommand.lazyStrafeToLinearHeading(drive, PARK_1, 10),
                RoadRunnerCommand.lazyStrafeToLinearHeading(drive, PARK_2, 2)
        );*/
        // END DEBUG
        /*return new SequentialCommandGroupFix(
                new InstantCommand(() -> armSubsystem.applyIntakeState(IntakeState.INTAKE)),
                new RepeatCommand(
                        new SequentialCommandGroupFix(
                                new InstantCommand(() -> telemetry.addData("sample in intake", armSubsystem.hasSampleInIntake())),
                                new InstantCommand(telemetry::update)
                        )
                )
        );*/

        // Compose some commands
        Command preloadScore = composeScoreCommand(BASKET_POSE_FIRST);

        Command park = composeParkCommand(PARK_1, PARK_2);


        // Compose final command
        if (doLeftSample) {
            return new SequentialCommandGroupFix(
                    new WaitCommand(100),
                    preloadScore,
                    composeSampleCommand(BASKET_POSE, SAMPLE_1_APP, SAMPLE_1_INT, false),
                    composeSampleCommand(BASKET_POSE, SAMPLE_2_APP, SAMPLE_2_INT, true),
                    composeThirdSampleCommand(BASKET_POSE, SAMPLE_3_APP1, SAMPLE_3_APP2, SAMPLE_3_INT),
                    park
            );
        } else {
            return new SequentialCommandGroupFix(
                    new WaitCommand(100),
                    preloadScore,
                    composeSampleCommand(BASKET_POSE, SAMPLE_1_APP, SAMPLE_1_INT, false),
                    composeSampleCommand(BASKET_POSE, SAMPLE_2_APP, SAMPLE_2_INT, true),
                    park
            );
        }
    }

    private Command composeSampleCommand(Pose2d BASKET_POSE, Pose2d APPROACH_POSE, Pose2d INTAKE_POSE, boolean BASKET_SCAN) {
        /*Command approach = new RoadRunnerCommand(drive.actionBuilder(BASKET_POSE)
                // Approach spike mark sample
                .strafeToLinearHeading(APPROACH_POSE.position, APPROACH_POSE.heading)
                .build());*/
        Command approach = RoadRunnerCommand.strafeToLinearHeading(drive, APPROACH_POSE);

        /*Command intake = new RoadRunnerCommand(drive.actionBuilder(APPROACH_POSE)
                // Intake spike mark sample
                .strafeToLinearHeading(INTAKE_POSE.position, INTAKE_POSE.heading)
                .build());*/
        Command intake = RoadRunnerCommand.strafeToLinearHeading(drive, INTAKE_POSE);

        Command score = composeScoreCommand(BASKET_POSE);
        //Command score = BASKET_SCAN ? composeScoreScanCommand(BASKET_POSE) : composeScoreCommand(BASKET_POSE);
        Command lowerArm = lowerArmSafely();

        return new SequentialCommandGroupFix(
                new InstantCommand(() -> lowerArm.schedule(false)),
                new ParallelRaceGroup(
                        new WaitUntilCommand(() -> false),
                        new SequentialCommandGroupFix(
                                approach,
                                intake
                        ),
                        new WaitUntilCommand(armSubsystem::hasSampleInIntake)
                ),
                new InstantCommand(() -> cancelIfScheduled(lowerArm)),
                armSubsystem.automaticZeroRotationCommand(),
                new InstantCommand(() -> offsetPoseIf(BASKET_SCAN)),
                score
        );
    }

    private void offsetPoseIf(boolean cond) {
        if (cond) drive.pose = new Pose2d(new Vector2d(drive.pose.position.x-3, drive.pose.position.y+3), drive.pose.heading);
    }

    private Command composeThirdSampleCommand(Pose2d BASKET_POSE, Pose2d APPROACH_POSE_1, Pose2d APPROACH_POSE_2, Pose2d INTAKE_POSE) {
        /*Command approach1 = new RoadRunnerCommand(drive.actionBuilder(BASKET_POSE)
                // Approach spike mark sample (first part)
                .strafeToLinearHeading(APPROACH_POSE_1.position, APPROACH_POSE_1.heading)
                .build());*/
        Command approach1 = RoadRunnerCommand.strafeToLinearHeading(drive, APPROACH_POSE_1);

        /*Command approach2 = new RoadRunnerCommand(() -> drive.actionBuilder(drive.pose)
                // Approach spike mark sample (second part)
                .strafeToLinearHeading(APPROACH_POSE_2.position, APPROACH_POSE_2.heading)
                .build());*/
        Command approach2 = RoadRunnerCommand.strafeToLinearHeading(drive, APPROACH_POSE_2);

        /*Command intake = new RoadRunnerCommand(drive.actionBuilder(APPROACH_POSE_2)
                // Intake spike mark sample
                .strafeToLinearHeading(INTAKE_POSE.position, INTAKE_POSE.heading)
                .build());*/
        Command intake = RoadRunnerCommand.strafeToLinearHeading(drive, INTAKE_POSE);

        Command score = composeScoreCommand(BASKET_POSE);
        Command yieldArmSafe = new WaitUntilCommand(() -> armSubsystem.getRotationPosition() < 0.4);
        Command lowerArm = lowerArmSafely();

        return new SequentialCommandGroupFix(
                new InstantCommand(() -> lowerArm.schedule(false)),
                new ParallelRaceGroup(
                        new WaitUntilCommand(() -> false),
                        new SequentialCommandGroupFix(
                                new ParallelDeadlineGroup(
                                        yieldArmSafe,
                                        approach1
                                ),
                                approach2,
                                intake
                        ),
                        new WaitUntilCommand(armSubsystem::hasSampleInIntake)
                ),
                new InstantCommand(() -> cancelIfScheduled(lowerArm)),
                armSubsystem.automaticZeroRotationCommand(),
                //new InstantCommand(() -> armSubsystem.applyNamedPosition("compact")),
                RoadRunnerCommand.lazyTurnTo(drive, Math.toRadians(85), Math.toRadians(10)),
                score
        );
    }

    private Command lowerArmSafely() {
        return new SequentialCommandGroupFix(
                new WaitUntilCommand(() -> drive.pose.heading.toDouble() > -Math.PI / 2), // This works because of angle wrapping
                new WaitCommand(400),
                new InstantCommand(() -> armSubsystem.applyNamedPosition("intake ground-high up")),
                new WaitUntilCommand(() -> drive.pose.heading.toDouble() < Math.PI * 2 / 3),
                new InstantCommand(() -> armSubsystem.applyNamedPosition("intake ground-high down"))
                //new WaitUntilCommand(() -> armSubsystem.isRotationAtTargetPosition(500))
        );
    }

    private Command composeScoreCommand(Pose2d BASKET_POSE) {
        return new SequentialCommandGroupFix(
                new InstantCommand(() -> armSubsystem.applyIntakeState(IntakeState.STOPPED)),
                new InstantCommand(() -> armSubsystem.applyNamedPosition("basket high")),
                new RoadRunnerCommand(() -> drive.actionBuilder(drive.pose)
                        .strafeTo(BASKET_POSE.position).build()),
                new WaitUntilCommand(() -> armSubsystem.isArmAtTargetPosition(20, 30, 30)),
                new RoadRunnerCommand(() -> drive.actionBuilder(drive.pose)
                        .turnTo(BASKET_POSE.heading).build()),
                new WaitCommand(BASKET_DELAY_MS),
                new InstantCommand(() -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE)),
                new WaitCommand(OUTTAKE_DELAY_MS)
        );
    }

    /*private Command composeScoreScanCommand(Pose2d BASKET_POSE) {
        return new SequentialCommandGroupFix(
                new InstantCommand(() -> armSubsystem.applyIntakeState(IntakeState.STOPPED)),
                new InstantCommand(() -> armSubsystem.applyNamedPosition("basket high")),
                new ParallelRaceGroup(
                        new WaitUntilCommand(() -> false),
                        new WaitUntilCommand(drive::didLastPoseEstUseVision),
                        new RoadRunnerCommand(drive.actionBuilder(drive.pose).turn(Math.PI).build())
                ),
                new RoadRunnerCommand(() -> drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(BASKET_POSE.position, Math.PI).build()),
                new WaitUntilCommand(() -> armSubsystem.isArmAtTargetPosition(20, 30, 30)),
                new RoadRunnerCommand(() -> drive.actionBuilder(drive.pose)
                        .turnTo(BASKET_POSE.heading).build()),
                new WaitCommand(BASKET_DELAY_MS),
                new InstantCommand(() -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE)),
                new WaitCommand(OUTTAKE_DELAY_MS)
        );
    }*/

    /*private Command composeScoreCommandArmless(Pose2d BASKET_POSE) {
        Vector2d BASKET_FIRST = new Vector2d(BASKET_POSE.position.x+12, BASKET_POSE.position.y);
        return new SequentialCommandGroupFix(
                new RoadRunnerCommand(() -> drive.actionBuilder(drive.pose)
                        .strafeTo(BASKET_POSE.position)
                        .build()),
                new RoadRunnerCommand(() -> drive.actionBuilder(drive.pose)
                        .turnTo(BASKET_POSE.heading).build()),
                new WaitCommand(BASKET_DELAY_MS),
                new WaitCommand(OUTTAKE_DELAY_MS)
        );
    }*/

    private Command composeParkCommand(Pose2d PARK_1, Pose2d PARK_2) {
        return new SequentialCommandGroupFix(
                // Park in ascent zone and achieve Level 1 Ascent
                new InstantCommand(() -> armSubsystem.applyIntakeState(IntakeState.STOPPED)),
                blockIfTimeout(2), // stop if not enough time left
                new ParallelCommandGroup(
                        new SequentialCommandGroupFix(
                                new WaitUntilCommand(() -> drive.pose.heading.toDouble() > -Math.PI / 2), // This works because of angle wrapping
                                new InstantCommand(() -> armSubsystem.applyNamedPosition("ascent level 1"))
                        ),
                        new SequentialCommandGroupFix(
                                RoadRunnerCommand.lazyStrafeToLinearHeading(drive, PARK_1, 10),
                                new ParallelCommandGroup(
                                        RoadRunnerCommand.lazyStrafeToLinearHeading(drive, PARK_2, 2),
                                        new SequentialCommandGroupFix(
                                                waitForTimeRemaining(0.2),
                                                new InstantCommand(() -> armSubsystem.setRotationPower(-0.05))
                                        )
                                )

                        )
                )
        );
    }

    private void cancelIfScheduled(Command command) {
        if (command.isScheduled()) command.cancel();
    }

    private Command waitForTimeRemaining(double time) {
        return new WaitUntilCommand(() -> 30 - getRuntime() < time);
    }

    private Command blockIfTimeout(double requiredTime) {
        return new WaitUntilCommand(() -> 30 - getRuntime() > requiredTime);
    }
}
