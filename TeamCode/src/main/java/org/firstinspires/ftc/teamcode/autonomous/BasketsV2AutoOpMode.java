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
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.helper.IntakeState;
import org.firstinspires.ftc.teamcode.helper.DriverPrompter;
import org.firstinspires.ftc.teamcode.helper.ResetZeroState;
import org.firstinspires.ftc.teamcode.helper.RoadRunnerCommand;
import org.firstinspires.ftc.teamcode.helper.localization.Localizers;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystemRRVision;
import org.firstinspires.ftc.teamcode.subsystems.VisionPortalSubsystem;
import org.jetbrains.annotations.NotNull;

/*
 * This file contains a simple example "OpMode" for driving a robot.
 * An OpMode (Operation Mode) is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The OpMode to use can be selected on the FTC Driver Station.
 *
 * Hardware names are listed in the Constants file. You must correctly set all hardware names for each
 * motor/servo/etc. in the Driver Station for the code to find your devices.
 *
 * This code goes from the starting position (second tile, ~40 in. from the left wall and facing away from the back wall),
 * scores a preload in the high net, collects and scores a few spike mark samples in the high net,
 * and then goes to park in the ascent zone.
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

@Config
@Autonomous(name="Baskets V2 Auto-OpMode (4 samples)", group="Autonomous OpMode")
public class BasketsV2AutoOpMode extends CommandOpMode {

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
        visionPortalSubsystem = new VisionPortalSubsystem(hardwareMap);

        // Initialize MecanumDrive at a particular pose
        Pose2d initialPose = legacyTransform(new Pose2d(0, 36, 0));
        drive = new DriveSubsystemRRVision(hardwareMap, visionPortalSubsystem,
                Localizers.ENCODERS_WITH_VISION, initialPose);

        // Generate path
        Action path = getBasketsV2(drive, armSubsystem);


        // Schedule commands

        // When this opmode is stopped, mark subsystems to not zero again once the next opmode (teleop) begins
        new Trigger(this::isStopRequested).whenActive(this::markToNotZeroWithPose);

        // Execute autonomous routine
        new RoadRunnerCommand(path).schedule(false);

        // At all times, update the telemetry log
        drive.new TelemetryLoggerCommand(telemetry).schedule(false);


        // Get alliance
        // This must be done last, as it waits for user input (which could take until the end of init)
        drive.setIsBlueAlliance(DriverPrompter.queryAlliance(this));
    }

    private void markToNotZeroWithPose() {
        ResetZeroState.markToNotZeroOnInit(drive.pose.position, drive.pose.heading.toDouble());
    }

    private Action getBasketsV2(DriveSubsystemRRVision drive, ArmSubsystem armSubsystem) {
        final Vector2d BASKET_POS = legacyTransform(new Vector2d(7.75, 63));
        final Pose2d BASKET_POSE = legacyTransform(new Pose2d(7.75, 63, Math.PI * 3 / 4));

        final Pose2d KEYPOINT_1 = legacyTransform(new Pose2d(21, 62, 0));
        final Pose2d KEYPOINT_2 = legacyTransform(new Pose2d(21.5, 71, 0));
        final Pose2d KEYPOINT_3 = legacyTransform(new Pose2d(40, 54, Math.PI / 2));
        final Pose2d KEYPOINT_3B = legacyTransform(new Pose2d(40, 58, Math.PI / 2));
        final Pose2d KEYPOINT_4 = legacyTransform(new Pose2d(52, 48, 0));
        final Pose2d KEYPOINT_5 = legacyTransform(new Pose2d(52, 30, -Math.PI / 2));

        final double INTAKE_DELAY = 0.3;
        final double OUTTAKE_DELAY = 0.3;

        return drive.actionBuilder(legacyTransform(new Pose2d(0, 36, 0)))
                // Score preload
                .afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                .afterTime(0.0, () -> armSubsystem.applyNamedPosition("basket high"))
                .strafeTo(BASKET_POS)
                .stopAndAdd(armSubsystem.yieldForRaiseTarget())
                .stopAndAdd(armSubsystem.yieldForRotationTarget())
                .turnTo(BASKET_POSE.heading)
                .afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .waitSeconds(OUTTAKE_DELAY)

                // Score right spike mark sample
                .afterTime(0.5, () -> armSubsystem.applyNamedPosition("intake ground-high"))
                .strafeToLinearHeading(KEYPOINT_1.position, KEYPOINT_1.heading)
                .stopAndAdd(armSubsystem.yieldForRotationTarget(55))
                .stopAndAdd(() -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                .waitSeconds(INTAKE_DELAY)
                .stopAndAdd(() -> armSubsystem.applyNamedPosition("basket high"))
                .stopAndAdd(armSubsystem.yieldForRotationTarget())
                .strafeToLinearHeading(BASKET_POSE.position, BASKET_POSE.heading)
                .stopAndAdd(() -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .waitSeconds(OUTTAKE_DELAY)

                // Score center spike mark sample
                .afterTime(0.5, () -> armSubsystem.applyNamedPosition("intake ground-high"))
                .strafeToLinearHeading(KEYPOINT_2.position, KEYPOINT_2.heading)
                .stopAndAdd(armSubsystem.yieldForRotationTarget(55))
                .stopAndAdd(() -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                .waitSeconds(INTAKE_DELAY)
                .stopAndAdd(() -> armSubsystem.applyNamedPosition("basket high"))
                .stopAndAdd(armSubsystem.yieldForRotationTarget())
                .strafeToLinearHeading(BASKET_POSE.position, BASKET_POSE.heading)
                .stopAndAdd(() -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .waitSeconds(OUTTAKE_DELAY)

                // Score left spike mark sample
                .afterTime(0.5, () -> armSubsystem.applyNamedPosition("intake ground-high"))
                .strafeToLinearHeading(KEYPOINT_3.position, KEYPOINT_3.heading)
                .stopAndAdd(armSubsystem.yieldForRotationTarget(55))
                .strafeToLinearHeading(KEYPOINT_3B.position, KEYPOINT_3B.heading)
                .waitSeconds(0.1)
                .stopAndAdd(() -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                .waitSeconds(INTAKE_DELAY)
                .strafeToLinearHeading(KEYPOINT_3.position, KEYPOINT_3.heading)
                .stopAndAdd(blockIfTimeout(3.0)) // If there aren't at least 3 seconds remaining at this point, the routine pauses indefinitely.
                .stopAndAdd(() -> armSubsystem.applyNamedPosition("basket high"))
                .stopAndAdd(armSubsystem.yieldForRotationTarget())
                .strafeToLinearHeading(BASKET_POSE.position, BASKET_POSE.heading)
                .stopAndAdd(() -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .waitSeconds(OUTTAKE_DELAY)

                // Park in ascent zone and achieve Level 1 Ascent
                .stopAndAdd(blockIfTimeout(2.0))
                .afterTime(0.5, () -> armSubsystem.applyNamedPosition ("ascent level 1"))
                .strafeToLinearHeading(KEYPOINT_4.position, KEYPOINT_4.heading)
                .strafeToLinearHeading(KEYPOINT_5.position, KEYPOINT_5.heading)
                .afterTime(0.0, () -> armSubsystem.setRotationPower(0.0))
                .build();
    }

    private Vector2d legacyTransform(Vector2d vec) {
        //return new Vector2d(-(vec.y - 8), vec.x - 61);
        return vec;
        //return isBlueAlliance ? vec : vec.times(-1);
    }

    private Pose2d legacyTransform(Pose2d pose) {
        //return new Pose2d(legacyTransform(pose.position), pose.heading.plus(Math.PI / 2));
        return pose;
        //return new Pose2d(legacyTransform(pose.position), isBlueAlliance ? pose.heading : pose.heading.plus(Math.PI));
    }

    private double adaptAngle(double angle) {
        return angle;
        //return angle + (isBlueAlliance ? 0 : Math.PI);
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
