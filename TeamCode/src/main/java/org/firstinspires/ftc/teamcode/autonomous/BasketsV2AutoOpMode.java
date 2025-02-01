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
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.helper.IntakeState;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDriveTune3;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
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
public class BasketsV2AutoOpMode extends LinearOpMode {

    // Hardware Variables
    private MecanumDriveTune3 drive;
    private ArmSubsystem armSubsystem;


    // This is run when the "INIT" button is pressed on the Driver Station.
    @Override
    public void runOpMode() {
        // Reset zero state; autonomous opmodes should always zero the robot
        ResetZeroState.resetZeroState();
        // Initialize MecanumDrive at a particular pose
        Pose2d initialPose = new Pose2d(0, 36, 0);
        drive = new MecanumDriveTune3(hardwareMap, initialPose);
        // Initialize hardware
        armSubsystem = new ArmSubsystem(hardwareMap, 0.5, 0.5, 1.0);

        // Generate path
        Action path = getBasketsV2(drive, armSubsystem);


        // Wait until start
        waitForStart();
        resetRuntime();
        if (isStopRequested()) return;


        // Mark subsystems to not zero again once the next opmode begins (teleop)
        ResetZeroState.markToNotZeroOnInit(false);

        // Execute autonomous routine
        Actions.runBlocking(path);
    }

    private Action getBasketsV2(MecanumDriveTune3 drive, ArmSubsystem armSubsystem) {
        //final Vector2d BASKET_POS = legacyTransform(new Vector2d(7.75, 63));
        //final Pose2d BASKET_POSE = legacyTransform(new Pose2d(7.75, 63, Math.PI * 3 / 4));
        final Vector2d BASKET_POS = legacyTransform(new Vector2d(8.25, 63.5));
        final Pose2d BASKET_POSE = legacyTransform(new Pose2d(8.25, 63.5, Math.PI * 3 / 4));

        final Pose2d KEYPOINT_1 = legacyTransform(new Pose2d(21, 62, 0));
        final double KEYPOINT_1_TAN = adaptAngle(Math.atan2(KEYPOINT_1.position.x - BASKET_POS.x, KEYPOINT_1.position.y - BASKET_POS.y));
        final Pose2d KEYPOINT_2 = legacyTransform(new Pose2d(21.5, 71, 0));
        final Pose2d KEYPOINT_3 = legacyTransform(new Pose2d(40, 54, Math.PI / 2));
        final Pose2d KEYPOINT_3B = legacyTransform(new Pose2d(40, 58, Math.PI / 2));
        final Pose2d KEYPOINT_4 = legacyTransform(new Pose2d(52, 63.5, -Math.PI / 2));
        final Pose2d KEYPOINT_5 = legacyTransform(new Pose2d(62, 36, -Math.PI / 2));

        return drive.actionBuilder(legacyTransform(new Pose2d(0, 36, 0)))
                // Score preload
                .afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                .afterTime(0.0, () -> armSubsystem.applyNamedPosition("basket high"))
                .strafeTo(BASKET_POS)
                .stopAndAdd(armSubsystem.yieldForRaiseTarget())
                .stopAndAdd(armSubsystem.yieldForRotationTarget())
                .turnTo(BASKET_POSE.heading)
                .afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .waitSeconds(0.3)

                // Score right spike mark sample
                .setTangent(adaptAngle(0))
                .splineToSplineHeading(KEYPOINT_1, adaptAngle(Math.PI / 2))
                .stopAndAdd(() -> armSubsystem.applyNamedPosition("intake ground-high"))
                //.waitSeconds(1.2)
                .stopAndAdd(armSubsystem.yieldForRotationTarget(55))
                .afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                .waitSeconds(0.3)
                .stopAndAdd(() -> armSubsystem.applyNamedPosition("basket high"))
                .stopAndAdd(armSubsystem.yieldForRotationTarget())
                .setTangent(adaptAngle(-Math.PI / 2))
                .splineToSplineHeading(BASKET_POSE, adaptAngle(Math.PI))
                .afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .waitSeconds(0.3)

                // Score center spike mark sample
                .setTangent(adaptAngle(Math.PI / 2))
                .splineToSplineHeading(KEYPOINT_2, adaptAngle(Math.PI))
                .stopAndAdd(() -> armSubsystem.applyNamedPosition("intake ground-high"))
                //.waitSeconds(1.2)
                .stopAndAdd(armSubsystem.yieldForRotationTarget(55))
                .afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                .waitSeconds(0.3)
                .stopAndAdd(() -> armSubsystem.applyNamedPosition("basket high"))
                .stopAndAdd(armSubsystem.yieldForRotationTarget())
                .setTangent(adaptAngle(0))
                .splineToSplineHeading(BASKET_POSE, adaptAngle(-Math.PI / 2))
                .afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .waitSeconds(0.3)

                // Score left spike mark sample
                //.turnTo(adaptAngle(Math.PI / 4))
                //.stopAndAdd(() -> armSubsystem.applyNamedPosition("intake ground-high"))
                .afterTime(1.0, () -> armSubsystem.applyNamedPosition("intake ground-high"))
                .setTangent(adaptAngle(Math.PI / 4))
                .splineToSplineHeading(KEYPOINT_3, adaptAngle(Math.PI / 2))
                .stopAndAdd(armSubsystem.yieldForRotationTarget(55))
                .setTangent(adaptAngle(Math.PI))
                .splineToSplineHeading(KEYPOINT_3B, adaptAngle(Math.PI))
                .waitSeconds(0.1)
                .afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                .waitSeconds(0.3)
                .setTangent(adaptAngle(0))
                .splineToSplineHeading(KEYPOINT_3, adaptAngle(0))
                .stopAndAdd(blockIfTimeout(3.0)) // If there aren't at least 3 seconds remaining at this point, the routine pauses indefinitely.
                .stopAndAdd(() -> armSubsystem.applyNamedPosition("basket high"))
                .stopAndAdd(armSubsystem.yieldForRotationTarget())
                .setTangent(adaptAngle(-Math.PI / 2))
                .splineToSplineHeading(BASKET_POSE, adaptAngle(Math.PI * 5 / 4))
                .afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .waitSeconds(0.3)

                // Park in ascent zone and achieve Level 1 Ascent
                .stopAndAdd(blockIfTimeout(2.0))
                .afterTime(0.5, () -> armSubsystem.applyNamedPosition("specimen low"))
                .setTangent(adaptAngle(Math.PI / 2))
                .splineToSplineHeading(KEYPOINT_4, adaptAngle(Math.PI / 2))
                .stopAndAdd(armSubsystem.yieldForRaiseTarget())
                .setTangent(adaptAngle(Math.PI / 2))
                .splineToSplineHeading(KEYPOINT_5, adaptAngle(0))
                .afterTime(0.0, () -> armSubsystem.setRotationPower(0.0))
                .build();
    }

    private static Vector2d legacyTransform(Vector2d vec) {
        //return new Vector2d(-(vec.y - 8), vec.x - 61);
        return vec;
    }

    private static Pose2d legacyTransform(Pose2d pose) {
        //return new Pose2d(legacyTransform(pose.position), pose.heading.plus(Math.PI / 2));\
        return pose;
    }

    private static double adaptAngle(double angle) {
        return angle - Math.PI / 2;
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
