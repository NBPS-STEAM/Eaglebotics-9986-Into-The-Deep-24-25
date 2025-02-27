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
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.helper.IntakeState;
import org.firstinspires.ftc.teamcode.helper.ResetZeroState;
import org.firstinspires.ftc.teamcode.helper.RoadRunnerCommand;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDriveTune1;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

import java.lang.Math;
import java.util.Arrays;

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
@Autonomous(name="Baskets V1 Auto-OpMode (3 samples)", group="Autonomous OpMode")
@Disabled
public class BasketsV1AutoOpMode extends CommandOpMode {

    // Hardware Variables
    private MecanumDriveTune1 drive;
    private ArmSubsystem armSubsystem;


    // This is run when the "INIT" button is pressed on the Driver Station.
    @Override
    public void initialize() {
        // Reset zero state; autonomous opmodes should always zero the robot
        ResetZeroState.resetZeroState();
        // Initialize MecanumDrive at a particular pose
        Pose2d initialPose = new Pose2d(0, 36, 0);
        drive = new MecanumDriveTune1(hardwareMap, initialPose);
        // Initialize hardware
        armSubsystem = new ArmSubsystem(hardwareMap, Constants.ARM_ROTATION_POWER_AUTO, Constants.ARM_EXTENSION_POWER_AUTO, Constants.ARM_RAISE_POWER_AUTO);

        // Generate paths
        // TODO: splines when going from basket to sample
        // TODO: go straight from basket to sample (no approach strafe)?
        // TODO: park in corner zone? maybe alternate opmode
        final Vector2d AUTO_BASKET_POS = new Vector2d(7.75, 63);

        final VelConstraint fastVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        drive.kinematics.new WheelVelConstraint(50),
                        new AngularVelConstraint(Math.PI * 1.5)
                ));
        final AccelConstraint fastAccelConstraint = new ProfileAccelConstraint(-50, 50);
        final TurnConstraints fastTurnConstraints = new TurnConstraints(Math.PI * 1.5, -Math.PI * 1.5, Math.PI * 1.5);

        Action path = drive.actionBuilder(initialPose)
                // Score preload
                .afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                .afterTime(0.0, () -> armSubsystem.applyNamedPosition("basket high"))
                .strafeTo(AUTO_BASKET_POS)
                .stopAndAdd(armSubsystem.yieldForRaiseTarget())
                .turnTo(Math.PI * 3 / 4)
                //.waitSeconds(2)
                .afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .waitSeconds(0.4)

                // Score right spike mark sample
                .turnTo(0)
                .afterTime(0.0, () -> armSubsystem.applyNamedPosition("intake ground"))
                .strafeTo(new Vector2d(16, 54))
                .stopAndAdd(armSubsystem.yieldForRaiseTarget())
                .strafeTo(new Vector2d(22, 54))
                .afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                .waitSeconds(0.5)
                .afterTime(0.0, () -> armSubsystem.applyNamedPosition("basket high"))
                .strafeTo(AUTO_BASKET_POS)
                .stopAndAdd(armSubsystem.yieldForRaiseTarget())
                .turnTo(Math.PI * 3 / 4)
                //.waitSeconds(2)
                .afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .waitSeconds(0.4)

                // Score center spike mark sample
                .turnTo(0)
                .afterTime(0.0, () -> armSubsystem.applyNamedPosition("intake ground"))
                .strafeTo(new Vector2d(16, 66))
                .stopAndAdd(armSubsystem.yieldForRaiseTarget())
                .strafeTo(new Vector2d(22, 66))
                .afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                .waitSeconds(0.5)
                .afterTime(0.0, () -> armSubsystem.applyNamedPosition("basket high"))
                .strafeTo(AUTO_BASKET_POS)
                .stopAndAdd(armSubsystem.yieldForRaiseTarget())
                .turnTo(Math.PI * 3 / 4)
                //.waitSeconds(2)
                .afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .waitSeconds(0.4)

                // Achieve Level 1 Ascent
                .afterTime(0.5, () -> armSubsystem.applyNamedPosition("ascent level 1"))
                .strafeToLinearHeading(new Vector2d(68, 48), 0, fastVelConstraint, fastAccelConstraint)
                .strafeToLinearHeading(new Vector2d(68, 30), -Math.PI / 2, fastVelConstraint, fastAccelConstraint)
                .afterTime(0.0, () -> armSubsystem.setRotationPower(0.0))
                .build();

        // Wait until start (remnant from before this was command-based)
        //waitForStart();
        //if (isStopRequested()) return;


        // Execute autonomous routine
        new RoadRunnerCommand(path).schedule(true);

        // Mark subsystems to not zero again once the next opmode begins (teleop)
        new Trigger(this::isStopRequested).whenActive(this::markToNotZeroWithPose);

        // When the intake intakes a sample, stop the intake
        new Trigger(armSubsystem::shouldStopIntakeForSample).whenActive(armSubsystem::stopIntake);
    }

    private void markToNotZeroWithPose() {
        ResetZeroState.markToNotZeroOnInit(drive.pose.position, drive.pose.heading.toDouble());
    }
}
