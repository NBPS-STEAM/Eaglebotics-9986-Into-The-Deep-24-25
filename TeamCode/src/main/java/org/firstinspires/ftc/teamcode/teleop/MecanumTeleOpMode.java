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
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/*
 * This file contains an example "OpMode" for driving a robot.
 * An OpMode (Operation Mode) is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The OpMode to use can be selected on the FTC Driver Station.
 *
 * This code uses gamepad controls for the claws and four motors on the ground and applies preset
 * positions for the arm's rotation, extension, and wrist. The driving is field-centric, unless toggled off.
 *
 * Before starting this OpMode, the arm lift must be in its lowest position and the extension fully
 * retracted. The robot should also be facing forward, or else the field-centric driving will be off until reset.
 *
 * This code works by setting up the robot and then binding its functions to commands (controller buttons).
 * This is called command-based programming. It's about breaking your code into organized parts and
 * controlling them through command signals rather than directly coding each part of the robot.
 * In teleop code like this, command-based programming keeps complex code straightforward and organized.
 *
 * The drivetrain, arm assembly, and paper plane (drone) launcher are each managed by their own
 * "subsystem". Subsystems are smaller systems that make up parts of the overall system. The drive
 * subsystem has the code for driving. The arm subsystem has the code that manages what the arm is
 * doing and where to put it. Go figure.
 * Subsystems aren't just a part of command-based programming, but they're a big part of good programming.
 * Break your systems down into smaller parts. Coding is a balance between adding new features and
 * making sure that you can keep adding new features. Not writing all of your code in one place makes
 * sure that those new features won't get in the way of each other.
 *
 * coding with subsystems and commands isn't the most direct way of thinking, but it all comes
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
 * Normal mode:
 * D-pad up             |   Switch to bottom intaking position/increment collection position
 * D-pad down           |   Switch to bottom intaking position/decrement collection position
 * D-pad left           |   Open left claw just a little
 * D-pad right          |   Open right claw just a little
 * Left stick button    |   Toggle both claws
 * North (Y/Δ) button   |   Switch to bottom scoring position/increment place position (also sets drive to slow speed)
 * South (A/X) button   |   Switch to bottom scoring position/decrement place position (also sets drive to slow speed)
 * West (X/□) button    |   Toggle left claw
 * East (B/○) button    |   Toggle right claw
 * Right stick button   |   Toggle both claws
 * Left bumper          |   Open both claws just a little
 * Right bumper         |   Move to stow position
 * Start button         |   Toggle manual mode
 *
 * Manual mode: (this is for zeroing the rotation/extension in case of emergency *cough* 9987 *cough*)
 * D-pad up             |   Arm extension coarse adjustment up (extend arm fast)
 * D-pad down           |   Arm extension coarse adjustment down (retract arm fast)
 * D-pad left           |   Arm extension fine adjustment up (extend arm slow)
 * D-pad right          |   Arm extension fine adjustment down (retract arm slow)
 * Left stick button    |   Close both claws
 * North (Y/Δ) button   |   Arm rotation coarse adjustment up (lift arm up fast)
 * South (A/X) button   |   Arm rotation coarse adjustment down (let arm down fast)
 * West (X/□) button    |   Arm rotation fine adjustment up (lift arm up slow)
 * East (B/○) button    |   Arm rotation fine adjustment down (let arm down slow)
 * Right stick button   |   Open both claws
 * Left bumper          |   Zero extension motor
 * Right bumper         |   Zero rotation motor
 * Start button         |   Toggle manual mode
 * Note: there is no control of the wrist in manual mode. It will stay where it was last.
 *
 * Controller 2:
 * Left stick           |   Drive robot (strafe)
 * Right stick          |   Drive robot (rotate)
 * West (X/□) button    |   Set to slow drive speed
 * South (A/X) button   |   Set to medium drive speed
 * East (B/○) button    |   Set to fast drive speed
 * North (Y/Δ) button   |   Zero robot heading
 * Start button         |   Toggle field-centric driving
 *
 * Tips:
 * If you don't understand what a particular method does, then hover your mouse over it to get some info and a short description.
 * If you click on a variable/method, then all uses of it in the file will be highlighted.
 * Click on a variable/method and press Ctrl+B (Cmd+B on mac) to be taken to where it was originally declared.
 * In addition, do that twice for a list of everywhere that variable/method is used.
 * You can also do that by holding ctrl while you click on the variable/method.
 */

@TeleOp(name="Mecanum TeleOp Mode", group="Driver OpMode")
public class MecanumTeleOpMode extends CommandOpMode {
    @Override
    public void initialize() {
        // Initialize hardware
        GamepadEx armGamepad = new GamepadEx(gamepad1);
        GamepadEx baseGamepad = new GamepadEx(gamepad2);

        DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMap, Constants.DRIVE_POWER_MULTIPLIER);
        ArmSubsystem armSubsystem = new ArmSubsystem(hardwareMap);

        // Initialize arm for play
        armSubsystem.cycleSetPositionInState(ArmSubsystem.ArmState.STOW, 0);
        armSubsystem.applyWristPosition(1.0);

        // Custom triggers
        /* A trigger is what activates a command depending on whether it's true or false. When the
         arm subsystem is in its "manual" state, that should change the control bindings, so here a
         trigger is made that will activate while this is true. Instead of asking for a boolean,
         Trigger asks for a method that returns a boolean so that it can run the method to check if
         the value changes. This is a "lambda", which is a form of "Runnable".
         More information about how that works below. */
        Trigger manualStateTrigger = new Trigger(armSubsystem::isInManualState);
        Trigger notManualStateTrigger = manualStateTrigger.negate();

        Trigger scoreStateTrigger = new Trigger(armSubsystem::isInScoreState);

        // Get bindable buttons
        // These buttons are used multiple times, so it's quicker
        // to get them once and store them in a variable.
        Button armButtonA = armGamepad.getGamepadButton(GamepadKeys.Button.A);
        Button armButtonB = armGamepad.getGamepadButton(GamepadKeys.Button.B);
        Button armButtonX = armGamepad.getGamepadButton(GamepadKeys.Button.X);
        Button armButtonY = armGamepad.getGamepadButton(GamepadKeys.Button.Y);
        Button armButtonDpadUp = armGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        Button armButtonDpadDown = armGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);
        Button armButtonDpadLeft = armGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        Button armButtonDpadRight = armGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);

        /* Note: A "Runnable" is a special type of variable that, instead of a value, holds a method.
         Runnables can be both passed around as variables and run like methods. They are most commonly
         used as a method parameter so that a method can be given another method to run.
         Lambdas are a quick way to turn code into a Runnable. You can make code into a
         Runnable and then put it into a parameter, such as with the code below.
         There are several ways to write a lambda. The simplest way is to write this:
         () -> thing.code(x, y, z)
         In some cases, you can to this:
         () -> { thing.code1(x, y); thing.code2(z); }
         ...and maybe Android Studio give you a suggestion that simplifies it for you.
         i.e. new InstantCommand( () -> mySystem.doSomething(x, y, z) );
         creates a new InstantCommand that uses the mySystem.doSomething() method with the parameters
         (x, y, z). Again, this gives it the method itself, not the outcome of running the method.
         There's also a shorter way to write a lambda:
         thing::code
         i.e. new InstantCommand( mySystem::doSomething );
         creates a new InstantCommand that uses the mySystem.doSomething() method with no parameters.
         A lambda created like this will run a single method with no parameters.
         Something like: new InstantCommand( mySystem::doSomething );
         Would be identical to: new InstantCommand( () -> mySystem.doSomething() );
         Lambdas are used in the Button Bindings section below. */


        // Button bindings
        // To bind methods, the method is used to create a command that is bound to an action.

        // Reminder: unlike languages such as Python and JavaScript, a single line of code in Java ends
        // with a semicolon (;), NOT a line break. A single line of Java code can span multiple text lines.

        // Cycling through arm positions and states
        armButtonY.and(notManualStateTrigger)
        // Call the "and" method from armButtonY to combine it with notManualStateTrigger and create a new trigger which is active only while armButtonY AND notManualStateTrigger are active.
                .whenActive(new InstantCommand(() -> armSubsystem.cycleSetPositionInState(ArmSubsystem.ArmState.SCORE, 1)));
                // Create a new instant command, register it using that trigger's whenActive (command will be called when the trigger becomes active)
        // Repeat for all other face buttons
        armButtonA.and(notManualStateTrigger)
                .whenActive(new InstantCommand(() -> armSubsystem.cycleSetPositionInState(ArmSubsystem.ArmState.SCORE, -1)));
        armButtonDpadUp.and(notManualStateTrigger)
                .whenActive(new InstantCommand(() -> armSubsystem.cycleSetPositionInState(ArmSubsystem.ArmState.INTAKE, 1)));
        armButtonDpadDown.and(notManualStateTrigger)
                .whenActive(new InstantCommand(() -> armSubsystem.cycleSetPositionInState(ArmSubsystem.ArmState.INTAKE, -1)));
        armGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).and(notManualStateTrigger)
                .whenActive(new InstantCommand(() -> armSubsystem.cycleSetPositionInState(ArmSubsystem.ArmState.STOW, 0)));

        armGamepad.getGamepadButton(GamepadKeys.Button.START)
                .whenActive(new InstantCommand(armSubsystem::toggleManualState));
        scoreStateTrigger
                .whileActiveContinuous(new RunCommand(armSubsystem::alignWristToBackdrop));

        // Claw controls
        armButtonDpadLeft.and(notManualStateTrigger)
                .whenActive(new InstantCommand(armSubsystem::openPartlyLeftClaw));
        armButtonDpadRight.and(notManualStateTrigger)
                .whenActive(new InstantCommand(armSubsystem::openPartlyRightClaw));
        armButtonX.and(notManualStateTrigger)
                .whenActive(new InstantCommand(armSubsystem::toggleLeftClaw));
        armButtonB.and(notManualStateTrigger)
                .whenActive(new InstantCommand(armSubsystem::toggleRightClaw));

        armGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).and(notManualStateTrigger)
                .whenActive(new InstantCommand(armSubsystem::openPartlyLeftClaw))
                .whenActive(new InstantCommand(armSubsystem::openPartlyRightClaw));
        armGamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(armSubsystem::toggleBothClaws));
        armGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(armSubsystem::toggleBothClaws));

        // Manual mode controls
        armGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).and(manualStateTrigger)
                .whenActive(new InstantCommand(armSubsystem::zeroExtensionMotor));
        armGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).and(manualStateTrigger)
                .whenActive(new InstantCommand(armSubsystem::zeroRotationMotor));

        armButtonY.and(manualStateTrigger)
                .whenActive(new InstantCommand(() -> armSubsystem.setRotationMotorPower(Constants.MANUAL_ROTATION_COARSE_POWER)));
        armButtonA.and(manualStateTrigger)
                .whenActive(new InstantCommand(() -> armSubsystem.setRotationMotorPower(-Constants.MANUAL_ROTATION_COARSE_POWER)));
        armButtonX.and(manualStateTrigger)
                .whenActive(new InstantCommand(() -> armSubsystem.setRotationMotorPower(Constants.MANUAL_ROTATION_FINE_POWER)));
        armButtonB.and(manualStateTrigger)
                .whenActive(new InstantCommand(() -> armSubsystem.setRotationMotorPower(-Constants.MANUAL_ROTATION_FINE_POWER)));
        armButtonA.or(armButtonB).or(armButtonX).or(armButtonY).negate().and(manualStateTrigger)
                .whenActive(new InstantCommand(() -> armSubsystem.setRotationMotorPower(0)));

        armButtonDpadUp.and(manualStateTrigger)
                .whenActive(new InstantCommand(() -> armSubsystem.setExtensionMotorPower(Constants.MANUAL_EXTENSION_COARSE_POWER)));
        armButtonDpadDown.and(manualStateTrigger)
                .whenActive(new InstantCommand(() -> armSubsystem.setExtensionMotorPower(-Constants.MANUAL_EXTENSION_COARSE_POWER)));
        armButtonDpadLeft.and(manualStateTrigger)
                .whenActive(new InstantCommand(() -> armSubsystem.setExtensionMotorPower(Constants.MANUAL_EXTENSION_FINE_POWER)));
        armButtonDpadRight.and(manualStateTrigger)
                .whenActive(new InstantCommand(() -> armSubsystem.setExtensionMotorPower(-Constants.MANUAL_EXTENSION_FINE_POWER)));
        armButtonDpadUp.or(armButtonDpadDown).or(armButtonDpadLeft).or(armButtonDpadRight).negate().and(manualStateTrigger)
                .whenActive(new InstantCommand(() -> armSubsystem.setExtensionMotorPower(0)));

        notManualStateTrigger
                .whenActive(new InstantCommand(() -> armGamepad.gamepad.setLedColor(0, 0, 1, 120000)))
                .whenActive(new InstantCommand(() -> armGamepad.gamepad.rumbleBlips(1)))
                .whenInactive(new InstantCommand(() -> armGamepad.gamepad.setLedColor(0, 1, 0, 120000)))
                .whenInactive(new InstantCommand(() -> armGamepad.gamepad.rumbleBlips(1)));

        // Base driver controls
        baseGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(driveSubsystem::setSlowSpeed));
        baseGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(driveSubsystem::setMediumSpeed));
        baseGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(driveSubsystem::setFullSpeed));
        baseGamepad.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(driveSubsystem::toggleIsFieldCentric));
        baseGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(driveSubsystem::zeroHeading));

        // Set the default commands
        /* The default command is repeatedly run while no other commands are sent to the subsystem,
         so this will check the controller and run these things constantly.
         Since none of the commands above have been associated with a subsystem,
         these will always run constantly. */
        driveSubsystem.setDefaultCommand(new RunCommand(() -> driveSubsystem.drive(baseGamepad.getLeftY(), baseGamepad.getLeftX(), baseGamepad.getRightX()), driveSubsystem));
        armSubsystem.setDefaultCommand(new RunCommand(() -> armSubsystem.setExtensionServoPower(armGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - armGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)), armSubsystem));

        // Update telemetry every loop
        //schedule(new RunCommand(telemetry::update));
        // there's no telemetry in this opmode lolololol
    }
}
