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
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetrySubsystem;

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
 * Controller 1 (base driver):
 * Left stick           |   Drive robot (strafe)
 * Right stick          |   Drive robot (turn)
 * D-pad down button    |   Zero (reset) robot heading direction
 * West (X/□) button    |   Drive at fast speed
 * South (A/X) button   |   Drive at medium speed
 * East (B/○) button    |   Drive at slow speed
 *
 *
 * Controller 2 (arm driver):
 * South (A/X) button   |   Move arm to 'stow' set position
 * East (B/○) button    |   Move arm to 'intake' set position
 * West (X/□) button    |   Move arm to 'basket low' set position
 * North (Y/Δ) button   |   Move arm to 'basket high' set position
 * Start button         |   Move arm to 'hang part 1' set position, then press again to move to 'hang part 2'
 * Left stick button    |   Toggle intake
 * Right stick button   |   Toggle outtake
 *
 * D-pad up button      |   Manually extend arm up
 * D-pad down button    |   Manually extend arm down
 * D-pad right button   |   Manually rotate arm up
 * D-pad left button    |   Manually rotate arm down
 * Select + left bumper |   Zero (reset) extension motor
 * Select + right bumper|   Zero (reset) rotation motor
 *
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

    // Hardware Variables
    private GamepadEx baseGamepad;
    private GamepadEx armGamepad;

    private DriveSubsystem driveSubsystem;
    private ArmSubsystem armSubsystem;
    private TelemetrySubsystem telemetrySubsystem;


    // This is run when the "INIT" button is pressed on the Driver Station.
    @Override
    public void initialize() {
        // Initialize hardware
        baseGamepad = new GamepadEx(gamepad1);
        armGamepad = new GamepadEx(gamepad2);

        driveSubsystem = new DriveSubsystem(hardwareMap, Constants.DRIVE_POWER_MULTIPLIER);
        armSubsystem = new ArmSubsystem(hardwareMap);
        telemetrySubsystem = new TelemetrySubsystem(this, driveSubsystem, armSubsystem);

        // Initialize arm for play
        // These named set positions are set in the ArmSubsystem class.
        armSubsystem.applyNamedPosition("stow");

        // Disable telemetry?
        // Uncomment to disable reporting telemetry (live information about the robot) to the Driver Station.
        //telemetrySubsystem.disableTelemetry();


        // Button bindings
        // To bind methods, you create a command that is bound to an action.
        // ABXY corresponds to the Microsoft/XBOX layout, not Nintendo.

        // Reminder: unlike languages such as Python, a single line of code in Java ends with a
        // semicolon (;), NOT a line break. A single line of Java code can span multiple text lines.


        // Base Controls

        // Driving is done by the default command, which is set later in this method.

        // Reset Heading Direction
        bindToButtons(baseGamepad, driveSubsystem::zeroHeading, Button.DPAD_DOWN); // Zero (reset) robot heading direction

        // Drive Speeds
        bindToButtons(baseGamepad, driveSubsystem::setFullSpeed, Button.X); // Drive at fast speed
        bindToButtons(baseGamepad, driveSubsystem::setMediumSpeed, Button.A); // Drive at medium speed
        bindToButtons(baseGamepad, driveSubsystem::setSlowSpeed, Button.B); // Drive at slow speed


        // Arm Controls

        // Apply Set Positions
        bindToButtons(armGamepad, () -> armSubsystem.applyNamedPosition("stow"), Button.A); // Move arm to 'stow' set position
        bindToButtons(armGamepad, () -> armSubsystem.applyNamedPosition("intake"), Button.B); // Move arm to 'intake' set position
        bindToButtons(armGamepad, () -> armSubsystem.applyNamedPosition("basket low"), Button.X); // Move arm to 'basket low' set position
        bindToButtons(armGamepad, () -> armSubsystem.applyNamedPosition("basket high"), Button.Y); // Move arm to 'basket high' set position

        bindToButtons(armGamepad, () -> cyclePositions("hang part 1", "hang part 2"), Button.START); // Move arm to 'hang part 1' set position, then press again to move to 'hang part 2'

        // Intake Controls
        bindToButtons(armGamepad, armSubsystem::toggleIntakeIn, Button.LEFT_STICK_BUTTON); // Toggle intake
        bindToButtons(armGamepad, armSubsystem::toggleIntakeOut, Button.RIGHT_STICK_BUTTON); // Toggle outtake

        // Zero Arm Motors
        // These binds combine two triggers into one using and().
        // this turns the GamepadButton trigger into a regular Trigger. whenActive() is the equivalent to whenPressed() for a regular Trigger.
        bindToButtons(armGamepad, armSubsystem::zeroExtensionMotor, Button.BACK, Button.LEFT_BUMPER); // Zero (reset) extension motor
        bindToButtons(armGamepad, armSubsystem::zeroRotationMotor, Button.BACK, Button.RIGHT_BUMPER); // Zero (reset) rotation motor

        // Manual Arm Controls
        armGamepad.getGamepadButton(Button.DPAD_UP) // Manually extend arm up
                .whileActiveContinuous(new RunCommand(() -> armSubsystem.setExtensionTargetDistance(100)))
                .whenInactive(new InstantCommand(() -> armSubsystem.setExtensionTargetDistance(0)));

        armGamepad.getGamepadButton(Button.DPAD_DOWN) // Manually extend arm down
                .whileActiveContinuous(new RunCommand(() -> armSubsystem.setExtensionTargetDistance(-100)))
                .whenInactive(new InstantCommand(() -> armSubsystem.setExtensionTargetDistance(0)));

        armGamepad.getGamepadButton(Button.DPAD_RIGHT) // Manually rotate arm up
                .whileActiveContinuous(new RunCommand(() -> armSubsystem.setRotationTargetDistance(100)))
                .whenInactive(new InstantCommand(() -> armSubsystem.setRotationTargetDistance(0)));

        armGamepad.getGamepadButton(Button.DPAD_LEFT) // Manually rotate arm down
                .whileActiveContinuous(new RunCommand(() -> armSubsystem.setRotationTargetDistance(-100)))
                .whenInactive(new InstantCommand(() -> armSubsystem.setRotationTargetDistance(0)));


        // Default Commands

        // The default command of a subsystem is repeatedly run while no other commands are sent to the subsystem.
        // Since none of the commands above have been associated with a subsystem, this will always run constantly.
        driveSubsystem.setDefaultCommand(new RunCommand(() -> driveSubsystem.drive(baseGamepad), driveSubsystem));

        telemetrySubsystem.setDefaultCommand(new RunCommand(telemetrySubsystem::reportTelemetry, telemetrySubsystem));
    }


    // Helper Methods

    /**
     * Bind an action to run through an instant command when one or more buttons of a gamepad are pressed.
     */
    public void bindToButtons(GamepadEx gamepad, Runnable action, Button... buttons) {
        combineButtons(gamepad, buttons).whenActive(new InstantCommand(action));
    }
    // Button... is the same as writing Button[], but when you use the method you can type multiple
    // Buttons as if there were multiple parameters and they'll automatically be converted into an array.
    // i.e. bindToButtons(gamepad, action, Button.A, Button.B);

    /**
     * Combine multiple buttons of a gamepad into a single trigger.
     */
    public Trigger combineButtons(GamepadEx gamepad, Button... buttons) {
        Trigger composite = gamepad.getGamepadButton(buttons[0]);
        for (int i = 1; i < buttons.length; i++) composite = composite.and(gamepad.getGamepadButton(buttons[i]));
        return composite;
    }

    /**
     * Out of two arm positions, apply the first position, or apply the second position if already
     * in either of the two positions.
     */
    private void cyclePositions(String position1, String position2) {
        if (armSubsystem.getLastSetPosition().equals(position1) || armSubsystem.getLastSetPosition().equals(position2)) {
            armSubsystem.applyNamedPosition(position2);
        } else {
            armSubsystem.applyNamedPosition(position1);
        }
    }
}
