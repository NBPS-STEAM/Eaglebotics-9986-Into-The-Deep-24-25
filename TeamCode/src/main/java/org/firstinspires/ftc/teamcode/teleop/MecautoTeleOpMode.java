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

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.autonomous.AutoCommands;
import org.firstinspires.ftc.teamcode.helper.DriverPrompter;
import org.firstinspires.ftc.teamcode.helper.localization.Localizers;
import org.firstinspires.ftc.teamcode.subsystems.*;

import java.util.Collections;
import java.util.Set;
import java.util.function.DoubleSupplier;

/*
 * This file contains a "OpMode" for driving a robot.
 * An OpMode (Operation Mode) is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The OpMode to use can be selected on the FTC Driver Station.
 *
 * Hardware names are listed in the Constants file. You must correctly set all hardware names for each
 * motor/servo/etc. in the Driver Station for the code to find your devices.
 *
 * This code uses gamepad controls for a claw and four motors on the ground and applies preset positions for the arm's
 * rotation, extension, and wrist. The driving is field-centric. It makes use of a camera for vision recognition with
 * AprilTags for autonomous driving features.
 *
 * Before starting this OpMode, the arm lift must be in its lowest position and the extension fully retracted.
 * The robot must also be facing forward or have been facing forward at the start if this runs after an autonomous routine.
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
 * Left stick / Touchpad|   Drive robot (strafe)
 * Right stick/ Touchpad|   Drive robot (turn)
 * West (X/□) button    |   Drive at fast speed
 * South (A/X) button   |   Drive at medium speed
 * East (B/○) button    |   Drive at slow speed
 * Right trigger        |   Lower intake in submersible or toggle height for specimen scoring
 *
 * Left trigger         |   Automatically drive to score (an AprilTag must be visible)
 * Left bumper          |   Automatically drive to intake a specimen from the observation zone (specimen mode only) (an AprilTag must be visible)
 * Right bumper         |   Stop automatically driving (can also be done by moving either stick)
 *
 * D-pad down button    |   Zero (reset) robot heading direction
 * D-pad left button    |   Toggle specimen mode
 * D-pad up button      |   Toggle alliance color
 *
 * Start button         |   Manually forward retraction motor (tighten for going down)
 * Select button        |   Manually reverse retraction motor (release for going up)
 *
 * The controller rumbles a little while an AprilTag is visible to the robot and rumbles more during auto driving. This is required to start autonomous driving.
 * The alliance color must be correct for autonomous driving to work. The current alliance color is shown on the controller's LEDs.
 * Alliance color and specimen mode can also be set during init. This is preferred.
 *
 *
 * Controller 2 (arm driver):
 * REMEMBER:
 * IMPORTANT SAMPLE CONTROLS:   A, X, (D-RIGHT, B, RB)
 * IMPORTANT SPECIMEN CONTROLS: (A, X, B), (Y, D-UP, LS-UP)
 *
 * South (A/X) button   |   Move arm to 'compact' set position (zero rotation then rumble if pressed a second time)
 * East (B/○) button    |   Toggle outtake
 * West (X/□) button    |   Move arm to 'intake vertical' set position (outtakes if has sample)
 * North (Y/Δ) button   |   Move arm to 'intake ground' set position
 * Left bumper          |   Toggle intake
 * Right bumper         |   Unlock arm subsystem (set positions cannot be applied while locked)
 * D-pad up button      |   Move arm to 'specimen high' set position
 * D-pad down button    |   Move arm to 'specimen low' set position
 * D-pad right button   |   Move arm to 'basket high' set position (locks arm subsystem)
 * D-pad left button    |   Move arm to 'basket low' set position (locks arm subsystem)
 * Start button         |   Move arm to 'hang stage 1' position, then press again to move to 'hang stage 2' position
 * Select button        |   Move arm to 'stow' set position
 *
 * Left stick up        |   Manually rotate arm up
 * Left stick down      |   Manually rotate arm down
 * Guide + left stick up|   Manually extend arm out
 * Guide + left stick dn|   Manually extend arm in
 * Right stick up       |   Manually raise arm up
 * Right stick down     |   Manually raise arm down
 * Guide + right stck up|   Manually turn wrist up (puts the wrist at an offset that persists through set positions)
 * Guide + right stck dn|   Manually turn wrist down (puts the wrist at an offset that persists through set positions)
 * Guide + left bumper  |   Zero (reset) extension motor
 * Guide + right bumper |   Zero (reset) rotation motor
 * Guide + start button |   Zero (reset) raise motor
 *
 *
 * Tips:
 * If you don't understand what a particular method does, then hover your mouse over it to get some info and a short description.
 * If you click on a variable/method, then all uses of it in the file will be highlighted.
 * Click on a variable/method and press Ctrl+B (Cmd+B on mac) to be taken to where it was originally declared.
 * In addition, do that twice for a list of everywhere that variable/method is used.
 * You can also do that by holding ctrl while you click on the variable/method.
 */

@TeleOp(name="Mec-auto Tele-OpMode", group="Driver OpMode")
public class MecautoTeleOpMode extends CommandOpMode {

    // Variables
    private GamepadEx baseGamepad;
    private GamepadEx armGamepad;

    private DriveSubsystemRRVision driveSubsystem;
    private ArmSubsystem armSubsystem;
    private VisionPortalSubsystem visionPortalSubsystem;

    private AutoCommands autoCommands;

    private boolean isSpecimenMode = false;

    // This is run when the "INIT" button is pressed on the Driver Station.
    @Override
    public void initialize() {
        // Initialize hardware
        baseGamepad = new GamepadEx(gamepad1);
        armGamepad = new GamepadEx(gamepad2);

        visionPortalSubsystem = new VisionPortalSubsystem(hardwareMap, Constants.CAM_DO_STREAM);
        armSubsystem = new ArmSubsystem(hardwareMap);
        driveSubsystem = new DriveSubsystemRRVision(hardwareMap, visionPortalSubsystem,
                Localizers.ENCODERS_WITH_VISION, new Pose2d(0, 0, 0));

        driveSubsystem.setFullSpeed();

        autoCommands = new AutoCommands(driveSubsystem, armSubsystem);


        // Button bindings
        // To bind methods, you create a command that is bound to an action.
        // ABXY corresponds to the Microsoft/XBOX layout, not Nintendo.

        // Reminder: unlike languages such as Python, a single line of code in Java ends with a
        // semicolon (;), NOT a line break. A single line of Java code can span multiple text lines.


        // Base Controls

        // Driving is done by the default command, which is set later in this method.

        // Reset Heading Direction
        bindToButtons(baseGamepad, driveSubsystem::zeroDriverHeading, Button.DPAD_DOWN); // Zero (reset) robot heading direction

        // Drive Speeds
        bindToButtons(baseGamepad, driveSubsystem::setFullSpeed, Button.X); // Drive at fast speed
        bindToButtons(baseGamepad, driveSubsystem::setMediumSpeed, Button.A); // Drive at medium speed
        bindToButtons(baseGamepad, driveSubsystem::setSlowSpeed, Button.B); // Drive at slow speed

        // Driver Context Action
        getGamepadTrigger(armGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER).whenActive(armSubsystem.driverContextCommand()); // Lower intake in submersible or toggle height for specimen scoring

        // Autonomous Driving
        Command autoScoreSampleCommand = autoCommands.getScoreSampleCommand();
        Command autoScoreSpecimenCommand = autoCommands.getScoreSpecimenCommand();
        Command autoIntakeSpecimenCommand = autoCommands.getIntakeSpecimenCommand();

        Trigger visionTrigger = new Trigger(driveSubsystem::didLastPoseEstUseVision);
        Trigger specimenModeTrigger = new Trigger(() -> isSpecimenMode);
        Trigger notSpecimenModeTrigger = specimenModeTrigger.negate();

        Trigger ltVision = getGamepadTrigger(baseGamepad, GamepadKeys.Trigger.LEFT_TRIGGER).and(visionTrigger);
        Trigger lbVision = combineButtons(baseGamepad, Button.LEFT_BUMPER).and(visionTrigger);

        // Automatically drive to score (an AprilTag must be visible)
        // Scoring samples
        ltVision.and(notSpecimenModeTrigger)
                .and(new Trigger(() -> !autoScoreSampleCommand.isScheduled()))
                .and(new Trigger(() -> armSubsystem.isArmAtTargetPosition(50, 50, 50)))
                .whenActive(autoScoreSampleCommand);
        // Scoring specimens
        ltVision.and(specimenModeTrigger)
                .and(new Trigger(() -> !autoScoreSpecimenCommand.isScheduled()))
                .whenActive(autoScoreSpecimenCommand);

        // Automatically drive to intake a specimen from the observation zone (specimen mode only) (an AprilTag must be visible)
        lbVision.and(specimenModeTrigger)
                .and(new Trigger(() -> !autoIntakeSpecimenCommand.isScheduled()))
                .whenActive(autoIntakeSpecimenCommand);

        // Stop automatically driving (can also be done by moving either stick)
        combineButtons(baseGamepad, Button.RIGHT_BUMPER)
                .whileActiveContinuous(autoScoreSampleCommand::cancel)
                .whileActiveContinuous(autoScoreSpecimenCommand::cancel)
                .whileActiveContinuous(autoIntakeSpecimenCommand::cancel);

        getEitherStickBeyondTrigger(baseGamepad, Constants.STICK_INTERRUPT_DEADZONE_SQR)
                .whenActive(autoScoreSampleCommand::cancel)
                .whenActive(autoScoreSpecimenCommand::cancel)
                .whenActive(autoIntakeSpecimenCommand::cancel);

        // Retraction
        combineButtons(baseGamepad, Button.BACK) // Manually reverse retraction motor (release for going up)
                .whileActiveOnce(armSubsystem.getRunRetractPowerCommand(-1.0));
        combineButtons(baseGamepad, Button.START) // Manually forward retraction motor (tighten for going down)
                .whileActiveOnce(armSubsystem.getRunRetractPowerCommand(1.0));

        // Mode control
        bindToButtons(baseGamepad, this::toggleSpecimenMode, Button.DPAD_LEFT); // Toggle specimen mode
        bindToButtons(baseGamepad, this::toggleAlliance, Button.DPAD_UP); // Toggle alliance color


        // Arm Controls

        Trigger armGuide = new Trigger(() -> armGamepad.gamepad.guide);
        Trigger notArmGuide = armGuide.negate();

        // Apply Set Positions
        // These named set positions are defined in the ArmSubsystem class.
        combineButtons(armGamepad, Button.A).whenActive(andThenRumble(armSubsystem.compactOrZeroCommand(), armGamepad)); // Move arm to 'compact' set position (zero rotation then rumble if pressed a second time)
        combineButtons(armGamepad, Button.X).whenActive(armSubsystem.moveToSubmersibleIntakeCommand()); // Move arm to 'intake vertical' set position (outtakes if has sample)
        bindToButtons(armGamepad, () -> armSubsystem.applyNamedPosition("intake ground"), Button.Y); // Move arm to 'intake ground' set position
        bindToButtons(armGamepad, () -> armSubsystem.applyNamedPosition("stow"), Button.BACK); // Move arm to 'stow' set position

        bindToButtons(armGamepad, () -> armSubsystem.applyNamedPosition("specimen high"), Button.DPAD_UP); // Move arm to 'specimen high' set position
        bindToButtons(armGamepad, () -> armSubsystem.applyNamedPosition("specimen low"), Button.DPAD_DOWN); // Move arm to 'specimen low' set position
        bindToButtons(armGamepad, () -> armSubsystem.applyNamedPosition("basket high", true, true), Button.DPAD_RIGHT); // Move arm to 'basket high' set position (locks arm subsystem)
        bindToButtons(armGamepad, () -> armSubsystem.applyNamedPosition("basket low", true, true), Button.DPAD_LEFT); // Move arm to 'basket low' set position (locks arm subsystem)

        combineButtons(armGamepad, Button.START).and(notArmGuide).whenActive(armSubsystem.cycleHangCommand()); // Move arm to 'hang stage 1' position, then press again to move to 'hang stage 2' position

        combineButtons(armGamepad, Button.RIGHT_BUMPER).and(notArmGuide).whenActive(() -> armSubsystem.lockSetPosition(false)); // Unlock arm subsystem (set positions cannot be applied while locked)

        // Intake Controls
        combineButtons(armGamepad, Button.LEFT_BUMPER).and(notArmGuide).whenActive(armSubsystem::toggleIntake); // Toggle intake
        bindToButtons(armGamepad, armSubsystem::toggleOuttake, Button.B); // Toggle outtake

        // Zero Arm Motors
        combineButtons(armGamepad, Button.LEFT_BUMPER).and(armGuide).whenActive(armSubsystem::zeroExtensionMotor); // Zero (reset) extension motor
        combineButtons(armGamepad, Button.RIGHT_BUMPER).and(armGuide).whenActive(armSubsystem::zeroRotationMotor); // Zero (reset) rotation motor
        combineButtons(armGamepad, Button.START).and(armGuide).whenActive(armSubsystem::zeroRaiseMotor); // Zero (reset) raise motor

        // Manual Arm Controls
        getTriggerFromBiAnalog(armGamepad::getLeftY).and(notArmGuide) // Manually rotate arm up/down
                .whileActiveOnce(armSubsystem.getRunRotationPowerCommand(armGamepad::getLeftY));

        getTriggerFromBiAnalog(armGamepad::getLeftY).and(armGuide) // Manually extend arm up/down
                .whileActiveOnce(armSubsystem.getRunExtensionPowerCommand(armGamepad::getLeftY));

        getTriggerFromBiAnalog(armGamepad::getRightY).and(notArmGuide) // Manually raise arm up/down
                .whileActiveOnce(armSubsystem.getRunRaisePowerCommand(armGamepad::getRightY));

        getTriggerFromBiAnalog(armGamepad::getRightY).and(armGuide) // Manually turn wrist up/down (puts the wrist at an offset that persists through set positions)
                .whileActiveContinuous(() -> armSubsystem.changeWristOffset(armGamepad.getRightY() * 0.03));


        // Default/Automatic Commands

        // The default command of a subsystem is repeatedly run while no other commands are sent to the subsystem.
        // If no other commands have been associated with a subsystem, these will run constantly.

        // By default, drive the base using the gamepad
        driveSubsystem.setDefaultCommand(new DriveWithTouchCommand(baseGamepad));

        // When the intake has a sample, cycle intake intelligently
        new Trigger(armSubsystem::shouldStopIntakeForSample)
                .and(armSubsystem.notSmartIntakeScheduledT()).whenActive(() -> armSubsystem.runSmartIntakeCommand(determineIntakeStow()));

        // At all times, check whether to rumble or stop rumbling the base driver's controller
        // The rumble command is set up like this because rumble methods sometimes need to run multiple times to take effect.
        new RumbleControllerCommand(baseGamepad.gamepad).schedule(false);

        // At all times, update the telemetry log
        driveSubsystem.new TelemetryLoggerCommand(telemetry).schedule(false);


        // Driver Station

        // Wait until camera is ready (this will make it obvious if it doesn't activate)
        sleepForVisionPortal(this, visionPortalSubsystem, telemetry);

        // Get alliance
        // This must be done last, as it waits for user input (which could take until the end of init)
        setAlliance(DriverPrompter.queryAlliance(this), DriverPrompter.wasAllianceFromDriver());

        // Choose right trigger function
        setSpecimenMode(DriverPrompter.queryBoolean(this, false,
                "Start in specimen mode?", "Specimen mode"));

        telemetry.addLine("Mecauto Teleop Ready!");
        telemetry.update();
    }


    // Driving with Sticks and Touch Controls

    class DriveWithTouchCommand extends CommandBase {
        private final GamepadEx gamepadEx;
        private final Gamepad gamepad;

        private Float touchLastLY = null;
        private float touchThisLY = 0.0f;
        private float touchDeltaLY = 0.0f;
        private Float touchLastLX = null;
        private float touchThisLX = 0.0f;
        private float touchDeltaLX = 0.0f;
        private Float touchLastRX = null;
        private float touchThisRX = 0.0f;
        private float touchDeltaRX = 0.0f;

        public DriveWithTouchCommand(GamepadEx gamepadEx) {
            this.gamepadEx = gamepadEx;
            this.gamepad = gamepadEx.gamepad;
            addRequirements(driveSubsystem);
        }

        @Override
        public void initialize() {
            resetTouchDelta();
        }

        @Override
        public void execute() {
            updateTouchDelta();
            driveSubsystem.drive(gamepadEx.getLeftY() - (touchDeltaLY * Constants.DRIVE_TOUCHPAD_STRAFE_SENSITIVITY) / driveSubsystem.setPowerMultiplier,
                    gamepadEx.getLeftX() + (touchDeltaLX * Constants.DRIVE_TOUCHPAD_STRAFE_SENSITIVITY) / driveSubsystem.setPowerMultiplier,
                    gamepadEx.getRightX() - (touchDeltaRX * Constants.DRIVE_TOUCHPAD_TURN_SENSITIVITY) / driveSubsystem.setPowerMultiplier);
        }

        private void updateTouchDelta() {
            if (gamepad.touchpad_finger_1) {
                touchThisLY = gamepad.touchpad_finger_1_y;
                touchThisLX = gamepad.touchpad_finger_1_x;
                if (touchLastLY == null) touchDeltaLY = 0.0f;
                else touchDeltaLY = touchLastLY - touchThisLY;
                if (touchLastLX == null) touchDeltaLX = 0.0f;
                else touchDeltaLX = touchThisLX - touchLastLX;
                touchLastLY = touchThisLY;
                touchLastLX = touchThisLX;
            } else {
                touchLastLY = null;
                touchThisLY = 0.0f;
                touchDeltaLY = 0.0f;
                touchLastLX = null;
                touchThisLX = 0.0f;
                touchDeltaLX = 0.0f;
            }
            if (gamepad.touchpad_finger_2) {
                touchThisRX = gamepad.touchpad_finger_2_x;
                if (touchLastRX == null) touchDeltaRX = 0.0f;
                else touchDeltaRX = touchThisRX - touchLastRX;
                touchLastRX = touchThisRX;
            } else {
                touchLastRX = null;
                touchThisRX = 0.0f;
                touchDeltaRX = 0.0f;
            }
        }

        private void resetTouchDelta() {
            touchLastLY = null;
            touchThisLY = 0.0f;
            touchDeltaLY = 0.0f;
            touchLastLX = null;
            touchThisLX = 0.0f;
            touchDeltaLX = 0.0f;
            touchLastRX = null;
            touchThisRX = 0.0f;
            touchDeltaRX = 0.0f;
        }
    }


    // Helper Methods

    public static void sleepForVisionPortal(LinearOpMode opMode, VisionPortalSubsystem visionPortalSubsystem, Telemetry telemetry) {
        telemetry.addLine("Waiting for Camera...");
        telemetry.update();
        while (!visionPortalSubsystem.isVisionPortalStreaming()) {
            opMode.sleep(50);
        }
        telemetry.addData("Camera", "Ready!").setRetained(true);
        telemetry.update();
    }

    private void toggleSpecimenMode() {
        setSpecimenMode(!isSpecimenMode);
    }

    private void setSpecimenMode(boolean enabled) {
        isSpecimenMode = enabled;
        updateGamepadColors();
    }

    private void toggleAlliance() {
        setAlliance(!driveSubsystem.getIsBlueAlliance(), true);
    }

    private void setAlliance(boolean isBlueAlliance, boolean trusted) {
        driveSubsystem.setIsBlueAlliance(isBlueAlliance, trusted);
        updateGamepadColors();
    }

    private void updateGamepadColors() {
        double otherColor = isSpecimenMode ? 0.5 : 0.0;
        if (driveSubsystem.getIsBlueAlliance()) {
            gamepad1.setLedColor(otherColor, otherColor, 1.0, 200000);
            gamepad2.setLedColor(otherColor, otherColor, 1.0, 200000);
        } else {
            gamepad1.setLedColor(1.0, otherColor, otherColor, 200000);
            gamepad2.setLedColor(1.0, otherColor, otherColor, 200000);
        }
    }

    private String determineIntakeStow() {
        String position = armSubsystem.getLastSetPosition();
        if ("intake vertical".equals(position) || "intake vertical-down".equals(position)) {
            return "stow";
        }
        if ("intake ground".equals(position)) {
            return isSpecimenMode ? "specimen low" : "stow";
        }
        return null;
    }

    private Command andThenRumble(Command command, GamepadEx gamepad) {
        return command.andThen(new InstantCommand(() -> gamepad.gamepad.rumbleBlips(1)));
    }

    class RumbleControllerCommand extends CommandBase {
        private final Gamepad gamepad;

        public RumbleControllerCommand(Gamepad gamepad) {
            this.gamepad = gamepad;
        }

        @Override
        public void execute() {
            // Rumble intensely when something has taken the driver's control, else rumble softly if AprilTag is visible, else stop rumbling
            boolean defaultScheduled = driveSubsystem.getDefaultCommand().isScheduled();
            boolean hasDetections = visionPortalSubsystem.hasDetections();
            if (hasDetections || !defaultScheduled) {
                double strength = defaultScheduled ? 0.5 : 1.0;
                gamepad.rumble(strength, strength, 150000);
            } else {
                gamepad.stopRumble();
            }
        }
    }


    // Binding Methods

    /**
     * Bind an action to run through an instant command when one or more buttons of a gamepad are pressed.
     */
    public static void bindToButtons(GamepadEx gamepad, Runnable action, Button... buttons) {
        combineButtons(gamepad, buttons).whenActive(action); // action is put in an InstantCommand automatically by whenActive()
    }
    // Button... is the same as writing Button[], but when you use the method you can type multiple
    // Buttons as if there were multiple parameters and they'll automatically be converted into an array.
    // i.e. bindToButtons(gamepad, action, Button.A, Button.B);

    /**
     * Bind an action to run through an instant command when a button of a gamepad is pressed and another button of that same gamepad is not pressed.
     */
    public static void bindToButtonButNot(GamepadEx gamepad, Runnable action, Button button, Button notButton) {
        buttonButNot(gamepad, button, notButton).whenActive(action); // action is put in an InstantCommand automatically by whenActive()
    }

    /**
     * Combine multiple buttons of a gamepad into a single trigger.
     */
    public static Trigger combineButtons(GamepadEx gamepad, Button... buttons) {
        // This combines two triggers into one using and().
        // This turns the GamepadButton trigger into a regular Trigger.
        // whenActive() is the equivalent to whenPressed() for a regular Trigger.
        Trigger composite = gamepad.getGamepadButton(buttons[0]);
        for (int i = 1; i < buttons.length; i++) composite = composite.and(gamepad.getGamepadButton(buttons[i]));
        return composite;
    }

    /**
     * Get a trigger for a button, but active only while another button is not pressed.
     */
    public static Trigger buttonButNot(GamepadEx gamepad, Button button, Button notButton) {
        return gamepad.getGamepadButton(button).and(gamepad.getGamepadButton(notButton).negate());
    }

    /**
     * Bind an action to run through an instant command when a {@link DoubleSupplier}'s return value
     * is beyond the threshold for analog inputs such as joysticks ({@value Constants#ANALOG_COMMAND_THRESHOLD}).
     */
    public static void bindToAnalog(DoubleSupplier joystickSupplier, boolean whenAbove, Runnable action) {
        getTriggerFromAnalog(joystickSupplier, whenAbove).whenActive(action); // action is put in an InstantCommand automatically by whenActive()
    }

    /**
     * Get a {@link Trigger} that is active while a {@link DoubleSupplier}'s return value is beyond
     * the threshold for analog inputs such as joysticks ({@value Constants#ANALOG_COMMAND_THRESHOLD}).
     * <p>A {@code Supplier} is like a Runnable that returns a value. This method
     * expects a Supplier that returns a double: the value to be checked for the threshold.</p>
     */
    public static Trigger getTriggerFromAnalog(DoubleSupplier analogSupplier, boolean whenAbove) {
        if (whenAbove) {
            return new Trigger(() -> analogSupplier.getAsDouble() > Constants.ANALOG_COMMAND_THRESHOLD);
        } else {
            return new Trigger(() -> analogSupplier.getAsDouble() < -Constants.ANALOG_COMMAND_THRESHOLD);
        }
    }

    /**
     * Get a {@link Trigger} that is active while a {@link DoubleSupplier}'s return value is beyond the threshold
     * for analog inputs such as joysticks ({@value Constants#ANALOG_COMMAND_THRESHOLD}) IN EITHER DIRECTION.
     */
    public static Trigger getTriggerFromBiAnalog(DoubleSupplier analogSupplier) {
        return getTriggerFromAnalog(analogSupplier, true)
                .or(getTriggerFromAnalog(analogSupplier, false));
    }

    /**
     * Bind an action to run through an instant command when a gamepad's analog trigger is beyond
     * the threshold for analog inputs ({@value Constants#ANALOG_COMMAND_THRESHOLD}).
     */
    public static void bindToGamepadTrigger(GamepadEx gamepad, GamepadKeys.Trigger trigger, Runnable action) {
        getGamepadTrigger(gamepad, trigger).whenActive(action); // action is put in an InstantCommand automatically by whenActive()
    }

    /**
     * Get a {@link Trigger} that is active while a gamepad's analog trigger is beyond
     * the threshold for analog inputs ({@value Constants#ANALOG_COMMAND_THRESHOLD}).
     */
    public static Trigger getGamepadTrigger(GamepadEx gamepad, GamepadKeys.Trigger trigger) {
        return getTriggerFromAnalog(() -> gamepad.getTrigger(trigger), true);
    }

    /**
     * Get a {@link Trigger} that is active while either joystick on the gamepad
     * is farther from the center than the square root of thresholdSqr.
     */
    public static Trigger getEitherStickBeyondTrigger(GamepadEx gamepad, double thresholdSqr) {
        return new Trigger(() ->
                (gamepad.getLeftX() * gamepad.getLeftX() + gamepad.getLeftY() * gamepad.getLeftY() > thresholdSqr)
                || (gamepad.getRightX() * gamepad.getRightX() + gamepad.getRightY() * gamepad.getRightY() > thresholdSqr)
        );
    }
}
