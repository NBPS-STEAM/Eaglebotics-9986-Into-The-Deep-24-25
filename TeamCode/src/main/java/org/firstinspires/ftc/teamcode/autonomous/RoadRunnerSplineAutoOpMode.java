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
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.helper.IntakeState;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

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
 *
 * This version of the opmode makes use of splines, which allows the robot to turn while strafing. Splines are slightly risky.
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
@Autonomous(name="Road Runner + Splines Auto-OpMode", group="Autonomous OpMode")
public class RoadRunnerSplineAutoOpMode extends LinearOpMode {

    // Constants (for only this routine)
    private static final Vector2d BASKET_POS = new Vector2d(10, 64);

    // Hardware Variables
    private MecanumDrive drive;
    private ArmSubsystem armSubsystem;


    // This is run when the "INIT" button is pressed on the Driver Station.
    @Override
    public void runOpMode() {
        // Initialize MecanumDrive at a particular pose
        Pose2d initialPose = new Pose2d(0, 36, 0);
        drive = new MecanumDrive(hardwareMap, initialPose);
        // Initialize hardware
        armSubsystem = new ArmSubsystem(hardwareMap, Constants.ARM_ROTATION_POWER_AUTO, Constants.ARM_EXTENSION_POWER_AUTO, Constants.ARM_RAISE_POWER_AUTO);

        // Generate paths
        Action path = drive.actionBuilder(initialPose)
                // Score preload
                .afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                .afterTime(0.0, () -> armSubsystem.applyNamedPosition("basket high"))
                .strafeTo(BASKET_POS)
                .stopAndAdd(armSubsystem.YieldForRaiseTarget())
                .turnTo(Math.PI * 3 / 4)
                .afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .waitSeconds(0.2)

                // Score right spike mark sample
                .afterTime(0.3, () -> armSubsystem.applyNamedPosition("intake ground"))
                .splineTo(new Vector2d(16, 54), 0.0)
                .stopAndAdd(armSubsystem.YieldForRaiseTarget())
                .strafeTo(new Vector2d(24, 54))
                .afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                .waitSeconds(0.3)
                .afterTime(0.0, () -> armSubsystem.applyNamedPosition("basket high"))
                .strafeTo(BASKET_POS)
                .stopAndAdd(armSubsystem.YieldForRaiseTarget())
                .turnTo(Math.PI * 3 / 4)
                .afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .waitSeconds(0.2)

                // Score center spike mark sample
                .afterTime(0.3, () -> armSubsystem.applyNamedPosition("intake ground"))
                .splineTo(new Vector2d(16, 66), 0.0)
                .stopAndAdd(armSubsystem.YieldForRaiseTarget())
                .strafeTo(new Vector2d(24, 66))
                .afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                .waitSeconds(0.3)
                .afterTime(0.0, () -> armSubsystem.applyNamedPosition("basket high"))
                .strafeTo(BASKET_POS)
                .stopAndAdd(armSubsystem.YieldForRaiseTarget())
                .turnTo(Math.PI * 3 / 4)
                .afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .waitSeconds(0.2)

                // Park in ascent zone
                .afterTime(0.5, () -> armSubsystem.applyNamedPosition("stow"))
                .strafeTo(new Vector2d(68, 48))
                .strafeTo(new Vector2d(68, 38))
                .build();


        // Wait until start
        waitForStart();
        if (isStopRequested()) return;


        // Mark subsystems to not zero again once the next opmode begins (teleop)
        ResetZeroState.markToNotZeroOnInit(true);

        // Execute autonomous routine
        Actions.runBlocking(path);
    }
}
