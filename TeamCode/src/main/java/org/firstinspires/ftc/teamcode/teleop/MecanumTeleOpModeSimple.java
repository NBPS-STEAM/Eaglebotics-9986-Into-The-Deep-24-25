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

package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.helper.ArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystemAdvanced;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystemSimple;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/*
 * This file contains a simple example "OpMode" for driving a robot.
 * An OpMode (Operation Mode) is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The OpMode to use can be selected on the FTC Driver Station.
 *
 * Hardware names are listed in the Constants file. You must correctly set all hardware names for each
 * motor/servo/etc. in the Driver Station for the code to find your devices.
 *
 * This code uses gamepad controls for a claw and four motors on the ground and applies preset
 * positions for the arm's rotation, extension, and wrist. The driving is field-centric.
 *
 * Before starting this OpMode, the arm lift must be in its lowest position and the extension fully
 * retracted. The robot should also be facing forward, or else the field-centric driving will be off until reset.
 *
 * This code works by setting up the robot and then binding its functions to commands (controller buttons).
 * This is called command-based programming. It's about breaking your code into organized parts and
 * controlling them through command signals rather than directly coding each part of the robot.
 * In teleop code like this, command-based programming keeps complex code straightforward and organized.
 *
 * The drivetrain and arm assembly are each managed by their own "subsystem". Subsystems are smaller
 * systems that make up parts of the overall system. The drive subsystem has the code for driving.
 * The arm subsystem has the code that manages what the arm is doing and where to put it. Go figure.
 *
 * Subsystems aren't just a part of command-based programming, but they're a big part of good programming.
 * Break your systems down into smaller parts. Coding is a balance between adding new features and
 * making sure that you can keep adding new features. Not writing all of your code in one place makes
 * sure that those new features won't get in the way of each other.
 *
 * Coding with subsystems and commands isn't the most direct way to do it, but it all comes
 * together very clearly and leaves less room for mistakes. It's best to get used to it.
 *
 * For more information about the command system, please see the official documentation:
 * https://docs.ftclib.org/ftclib/command-base/command-system
 * Most of the pages in the "Command Base" section go into great detail about all the ways to use
 * commands; you don't have to understand all of it thoroughly. I think that the most important ones
 * to read are the pages on "Command System", "Subsystems", and "Convenience Features". The first few
 * things listed on that last one are very important.
 *
 *
 * The initialize() method runs when the "init" button is pressed on the Driver Station.
 * Everything afterward is handled by commands.
 *
 *
 *
 * Robot controls:
 * Controller 1:
 * Left stick           |   Drive robot (strafe)
 * Right stick          |   Drive robot (turn)
 * West (X/□) button    |   Move arm to high set position
 * South (A/X) button   |   Move arm to low set position
 * East (B/○) button    |   Toggle claw
 * North (Y/Δ) button   |   Zero (reset) robot heading direction
 *
 *
 * Tips:
 * If you don't understand what a particular method does, then hover your mouse over it to get some info and a short description.
 * If you click on a variable/method, then all uses of it in the file will be highlighted.
 * Click on a variable/method and press Ctrl+B (Cmd+B on mac) to be taken to where it was originally declared.
 * In addition, do that twice for a list of everywhere that variable/method is used.
 * You can also do that by holding ctrl while you click on the variable/method.
 */

@TeleOp(name="Mecanum TeleOp Mode (Simple)", group="Driver OpMode")
public class MecanumTeleOpModeSimple extends CommandOpMode {
    @Override
    public void initialize() {
        // Initialize hardware
        GamepadEx gamepad = new GamepadEx(gamepad1);

        DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMap, Constants.DRIVE_POWER_MULTIPLIER);
        ArmSubsystemSimple armSubsystem = new ArmSubsystemSimple(hardwareMap);

        // Initialize arm for play
        armSubsystem.goToStowPosition();
        armSubsystem.applyWristPosition(1.0);

        // Arm set positions
        // These are just variables which hold a couple numbers. That's what an ArmPosition is. It doesn't do anything by itself.
        // When one of these variables are given to ArmSubsystem, it reads those numbers as a position for parts of the arm to move to.
        ArmPosition highPosition = new ArmPosition(0.8, 0.8, 0.8);
        ArmPosition lowPosition = new ArmPosition(0.4, 0.4, 0.4);


        // Button bindings
        // To bind methods, you create a command that is bound to an action.
        // ABXY corresponds to the Microsoft/XBOX layout, not Nintendo.

        // Reminder: unlike languages such as Python, a single line of code in Java ends with a
        // semicolon (;), NOT a line break. A single line of Java code can span multiple text lines.

        // Arm binds
        gamepad.getGamepadButton(GamepadKeys.Button.X) // Move arm to high set position
                .whenActive(new InstantCommand(() -> armSubsystem.applyPosition(highPosition)));

        gamepad.getGamepadButton(GamepadKeys.Button.A) // Move arm to low set position
                .whenActive(new InstantCommand(() -> armSubsystem.applyPosition(lowPosition)));


        // Claw controls
        gamepad.getGamepadButton(GamepadKeys.Button.B) // Toggle claw
                .whenPressed(new InstantCommand(armSubsystem::toggleClaw));


        // Reset heading direction
        gamepad.getGamepadButton(GamepadKeys.Button.Y) // Zero (reset) robot heading direction
                .whenActive(new InstantCommand(driveSubsystem::zeroHeading));


        // Default commands
        // The default command of a subsystem is repeatedly run while no other commands are sent to the subsystem.
        // Since none of the commands above have been associated with a subsystem this will always run constantly.
        driveSubsystem.setDefaultCommand(new RunCommand(() -> driveSubsystem.drive(gamepad), driveSubsystem));

        // Update telemetry every loop
        //schedule(new RunCommand(telemetry::update));
        // there's no telemetry in this opmode lolololol
    }
}
