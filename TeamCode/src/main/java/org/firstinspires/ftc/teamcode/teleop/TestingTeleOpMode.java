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
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TestingSubsystem;

/*
 * This file contains a simple example "OpMode" for driving a robot.
 * An OpMode (Operation Mode) is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The OpMode to use can be selected on the FTC Driver Station.
 *
 * This code allows for manipulating individual motors and servos to test positions, power, direction, etc.
 * When the opmode starts, follow the instructions on the Driver Station to select and control a device.
 *
 * Hardware devices (motors/servos) are not initialized until they are selected and activated, and
 * will be closed when deactivated. When a motor is initialized, it will have its encoder value reset.
 *
 *
 * The initialize() method runs when the "init" button is pressed on the Driver Station.
 * Everything afterward is handled by commands.
 *
 *
 *
 * Robot controls:
 * Controller 1:
 * D-pad up/down        |   Move up or down the list to select a hardware device (motor/servo)
 * South (A/X) button   |   Activate the selected hardware device
 * East (B/○) button    |   Deactivate the currently selected hardware device
 * West (X/□) button    |   Toggle freezing the selected hardware device in place
 * North (Y/Δ) button   |   Zero the selected hardware device
 * Left stick up/down   |   Move the selected hardware device (set power or position depending on device)
 * Right stick up/down  |   Offset the selected hardware device (adjust power or position)
 * Left bumper          |   (Slot 1) Store the state of the selected hardware device (position/power)
 * Right bumper         |   (Slot 1) Load the stored state of and freeze the selected hardware device
 * Select + left bumper |   (Slot 2) Store the state of the selected hardware device (position/power)
 * Select + right bumper|   (Slot 2) Load the stored state of and freeze the selected hardware device
 * Start button         |   Reload the list of CONFIGURED devices (physically plugging/unplugging a device will not result in a change)
 *
 *
 * Tips:
 * If you don't understand what a particular method does, then hover your mouse over it to get some info and a short description.
 * If you click on a variable/method, then all uses of it in the file will be highlighted.
 * Click on a variable/method and press Ctrl+B (Cmd+B on mac) to be taken to where it was originally declared.
 * In addition, do that twice for a list of everywhere that variable/method is used.
 * You can also do that by holding ctrl while you click on the variable/method.
 */

@TeleOp(name="Testing Tele-OpMode", group="Driver OpMode")
public class TestingTeleOpMode extends CommandOpMode {

    // Hardware Variables
    private GamepadEx gamepad;
    private TestingSubsystem testingSubsystem;


    // This is run when the "INIT" button is pressed on the Driver Station.
    @Override
    public void initialize() {
        // Initialize hardware variables
        gamepad = new GamepadEx(gamepad1);
        testingSubsystem = new TestingSubsystem(hardwareMap, telemetry);

        // EMERGENCY GAMEPAD 2 DRIVING
        //DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMap);
        //driveSubsystem.setDefaultCommand(new RunCommand(() -> driveSubsystem.drive(gamepad2), driveSubsystem));


        // Controller bindings

        // Device Selection

        bindTo(Button.DPAD_UP, () -> testingSubsystem.changeSelection(-1)); // Move up the list to select a hardware device (motor/servo)
        bindTo(Button.DPAD_DOWN, () -> testingSubsystem.changeSelection(1)); // Move down the list to select a hardware device (motor/servo)
        bindTo(Button.A, testingSubsystem::activateSelected); // Activate the selected hardware device
        bindTo(Button.B, testingSubsystem::deactivateSelected); // Deactivate the currently selected hardware device

        // Device Control

        bindTo(Button.X, testingSubsystem::toggleFrozen); // Toggle freezing the selected hardware device in place
        bindTo(Button.Y, testingSubsystem::zeroSelected); // Zero the selected hardware device
        bindTo(Button.START, testingSubsystem::reloadDeviceList); // Reload the list of CONFIGURED devices (physically plugging/unplugging a device will not result in a change)
        bindTo(Button.LEFT_BUMPER, () -> testingSubsystem.storeStateSelected(0)); // (Slot 1) Store the state of the selected hardware device (position/power)
        bindTo(Button.RIGHT_BUMPER, () -> testingSubsystem.loadStateSelected(0)); // (Slot 1) Load the stored state of and freeze the selected hardware device
        bindToTwo(Button.BACK, Button.LEFT_BUMPER, () -> testingSubsystem.storeStateSelected(1)); // (Slot 2) Store the state of the selected hardware device (position/power)
        bindToTwo(Button.BACK, Button.RIGHT_BUMPER, () -> testingSubsystem.loadStateSelected(1)); // (Slot 2) Load the stored state of and freeze the selected hardware device

        // Scheduled Commands
        // Scheduled commands run periodically forever or until stopped/interrupted.
        // ^^ that's probably not right, but I don't have time right now to check
        // Read here for more info on the different types of commands: https://docs.ftclib.org/ftclib/command-base/command-system/convenience-commands

        schedule(new RunCommand(() -> testingSubsystem.moveSelected(gamepad.getLeftY()))); // Move the selected hardware device (set power or position depending on device)
        schedule(new RunCommand(() -> testingSubsystem.offsetSelected(gamepad.getRightY()))); // Offset the selected hardware device (adjust power or position)
        schedule(new RunCommand(testingSubsystem::reportTelemetry)); // Report telemetry
    }


    // Helper Methods

    /**
     * Bind an action to run through an instant command when a button is pressed on the gamepad.
     */
    public void bindTo(Button button, Runnable action) {
        gamepad.getGamepadButton(button).whenPressed(new InstantCommand(action));
    }

    /**
     * Bind an action to run through an instant command when two buttons are pressed on the gamepad.
     */
    public void bindToTwo(Button button1, Button button2, Runnable action) {
        gamepad.getGamepadButton(button1).and(gamepad.getGamepadButton(button2)).whenActive(new InstantCommand(action));
    }

    private static double squareSigned(double value) {
        // The joystick value is on the interval [-1, 1] (between -1 and 1).
        // Squaring the joystick value results in a value that is also within [-1, 1] but curves to be smaller as it approaches 0.
        // This makes precise adjustments easier without giving up the ability to reach high values.
        return value * value * (value < 0 ? -1 : 1);
    }
}
