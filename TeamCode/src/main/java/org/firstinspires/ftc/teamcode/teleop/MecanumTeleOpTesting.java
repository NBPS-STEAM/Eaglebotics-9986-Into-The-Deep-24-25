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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Calculations;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.helper.QuadMotorValues;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/*
 * NOTICE: This OpMode is not command-based, and is rather clunky. If you were wondering why
 * command-based is generally preferred... this is why.
 * Please see MecanumTeleOpMode first for an example of how to write better code.
 * (hint: don't put everything all in one file, especially not all in one method)
 *
 * This file contains an example "OpMode" for testing a robot.
 * An OpMode (Operation Mode) is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The OpMode to use can be selected on the FTC Driver Station.
 *
 * This class works like a main class.
 *
 * This code offers full control over each motor/servo on the robot and gives useful feedback on
 * positions, encoder values, etc. for testing and configuring the robot. Hypothetically it could be
 * used in a match, but if that's ever necessary, we are completely screwed.
 *
 * This code starts by setting up the robot, then repeats in a loop until the robot is turned off.
 *
 * Robot controls:
 *
 * Controller 1:
 * Start button         |   Toggle limp motors
 * Guide button         |   Cycle through control modes
     * mode 0 = control style 1, triggers change drive power
     * mode 1 = control style 1, triggers change stendo power
     * mode 2 = control style 1, triggers change lift power
     * mode 3 = control style 2, triggers change lift power
     * mode 4 = control style 3, triggers change lift power
 *
 * Control style 1 (full control):
 * Left stick           |   Drive robot (strafe)
 * Right stick          |   Drive robot (rotate)
 * D-pad up             |   Move wrist up
 * D-pad down           |   Move wrist down
 * D-pad left           |   Move left claw closed
 * D-pad right          |   Move right claw closed
 * North (Y/Δ) button   |   Rotate arm up
 * South (A/X) button   |   Rotate arm down
 * West (X/□) button    |   Move left claw open
 * East (B/○) button    |   Move right claw open
 * Right bumper         |   Extend stendo
 * Left bumper          |   Retract stendo
 * Right trigger down   |   Increase power for selected motor (decided by control mode)
 * Left trigger down    |   Decrease power for selected motor (decided by control mode)
 *
 * Control style 2 (arm target angle and individual motors):
 * Left stick           |   Set the arm's target angle (full range)
 * Right stick          |   Set the arm's target angle (limited to safe range)
 * West (X/□) button    |   Spin front left drive motor forward
 * North (Y/Δ) button   |   Spin front right drive motor forward
 * South (A/X) button   |   Spin back left drive motor forward
 * East (B/○) button    |   Spin back right drive motor forward
 * Right trigger down   |   Increase power for selected motor (decided by control mode)
 * Left trigger down    |   Decrease power for selected motor (decided by control mode)
 *
 * Control style 3 (zeroing motors):
 * West (X/□) button    |   Zero front left drive motor
 * North (Y/Δ) button   |   Zero front right drive motor
 * South (A/X) button   |   Zero back left drive motor
 * East (B/○) button    |   Zero back right drive motor
 * Right bumper         |   Zero arm rotation motor
 * Left bumper          |   Zero arm stendo motor
 * Right trigger down   |   Increase power for selected motor (decided by control mode)
 * Left trigger down    |   Decrease power for selected motor (decided by control mode)
 *
 * Tips:
 * If you don't understand what a particular method does, then hover your mouse over it to get some info and a short description.
 * If you click on a variable/method, then all uses of it in the file will be highlighted.
 * Click on a variable/method and press Ctrl+B (Cmd+B on mac) to be taken to where it was originally declared.
 * In addition, do that twice for a list of everywhere that variable/method is used.
 * You can also do that by holding ctrl while you click on the variable/method.
 */

@TeleOp(name="Mecanum TeleOp Testing Mode", group="Driver OpMode")
public class MecanumTeleOpTesting extends LinearOpMode {

    // Variables
    // The timer
    private final ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        // Declare variables that persist between loops
        // Debug modes
        boolean isLimp = false;
        int controlMode = 0;
        // Power levels
        double armStendoPower = Constants.ARM_EXTENSION_POWER;
        double armLiftPower = Constants.ARM_ROTATION_POWER;
        // The value of inputs at the last loop
        float lTLast = 0.0f;
        float rTLast = 0.0f;
        boolean startLast = false;
        boolean guideLast = false;


        // Initialize the motors/hardware. The names used here must correspond to the names set on the driver control station.
        DcMotor frontLeftDrive = hardwareMap.get(DcMotor.class, Constants.NAME_DRIVE_FL);
        DcMotor backLeftDrive = hardwareMap.get(DcMotor.class, Constants.NAME_DRIVE_BL);
        DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, Constants.NAME_DRIVE_FR);
        DcMotor backRightDrive = hardwareMap.get(DcMotor.class, Constants.NAME_DRIVE_BR);

        Servo leftClawServo = hardwareMap.get(Servo.class, Constants.NAME_CLAW_L);
        Servo rightClawServo = hardwareMap.get(Servo.class, Constants.NAME_CLAW_R);

        DcMotor armLiftMotor = hardwareMap.get(DcMotor.class, Constants.NAME_ARM_ROTATE);
        DcMotor armTravelMotor = hardwareMap.get(DcMotor.class, Constants.NAME_ARM_EXTEND_M);
        Servo armWristServo = hardwareMap.get(Servo.class, Constants.NAME_ARM_WRIST);

        // Configure motors
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);


        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMap, Constants.DRIVE_POWER_MULTIPLIER);

        leftClawServo.setDirection(Servo.Direction.REVERSE);
        rightClawServo.setDirection(Servo.Direction.FORWARD);

        armWristServo.setDirection(Servo.Direction.REVERSE);

        armTravelMotor.setDirection(DcMotor.Direction.FORWARD);
        armTravelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armTravelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armLiftMotor.setDirection((DcMotor.Direction.FORWARD));
        armLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ArmSubsystem armSubsystem = new ArmSubsystem(hardwareMap);
        armSubsystem.applyWristPosition(1.0);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Get current value of inputs now so they can't change during the loop
            // This only matters for inputs that are recorded at the last loop
            float rTNow = gamepad1.right_trigger;
            float lTNow = gamepad1.left_trigger;
            boolean startNow = gamepad1.start;
            boolean guideNow = gamepad1.guide;

            // Inputs
            if (controlMode == 4) {
                // Control style 3 (zeroing motors)
                // Drive motors
                if (gamepad1.a) {
                    backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if (gamepad1.b) {
                    backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if (gamepad1.x) {
                    frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if (gamepad1.y) {
                    frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                // Stendo motor
                if (gamepad1.left_bumper) {
                    armTravelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armTravelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                // Lift motor
                if (gamepad1.right_bumper) {
                    armLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            } else if (controlMode == 3) {
                // Control style 2 (arm target angle and individual motors)
                if (gamepad1.a) {
                    backLeftDrive.setPower(1);
                } else {
                    backLeftDrive.setPower(0);
                }
                if (gamepad1.b) {
                    backRightDrive.setPower(1);
                } else {
                    backRightDrive.setPower(0);
                }
                if (gamepad1.x) {
                    frontLeftDrive.setPower(1);
                } else {
                    frontLeftDrive.setPower(0);
                }
                if (gamepad1.y) {
                    frontRightDrive.setPower(1);
                } else {
                    frontRightDrive.setPower(0);
                }
            } else {
                // Control style 1 (full control)
                // Arm lift
                if (gamepad1.y) {
                    armLiftMotor.setPower(armLiftPower);
                } else if (gamepad1.a) {
                    armLiftMotor.setPower(-armLiftPower);
                } else {
                    armLiftMotor.setPower(0);
                }
                // Arm stendo
                if (gamepad1.right_bumper) {
                    armTravelMotor.setPower(armStendoPower);
                } else if (gamepad1.left_bumper) {
                    armTravelMotor.setPower(-armStendoPower);
                } else {
                    armTravelMotor.setPower(0);
                }
                // Arm wrist
                if (gamepad1.dpad_up) {
                    armWristServo.setPosition(armWristServo.getPosition() + 0.005);
                } else if (gamepad1.dpad_down) {
                    armWristServo.setPosition(armWristServo.getPosition() - 0.005);
                }
                // Claws
                if (gamepad1.dpad_left) {
                    leftClawServo.setPosition(leftClawServo.getPosition() + 0.005);
                } else if (gamepad1.x) {
                    leftClawServo.setPosition(leftClawServo.getPosition() - 0.005);
                }
                if (gamepad1.dpad_right) {
                    rightClawServo.setPosition(rightClawServo.getPosition() + 0.005);
                } else if (gamepad1.b) {
                    rightClawServo.setPosition(rightClawServo.getPosition() - 0.005);
                }
            }

            // Change testing modes
            if (startNow && !startLast) {
                if (!isLimp) {
                    armLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    armTravelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    isLimp = true;
                } else {
                    armLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    armTravelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    isLimp = false;
                }
            }
            if (guideNow && !guideLast) {
                controlMode++;
                if (controlMode > 4) controlMode = 0;
                if (controlMode == 3) {
                    armLiftMotor.setPower(armLiftPower);
                    armLiftMotor.setTargetPosition(armLiftMotor.getCurrentPosition());
                    armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else {
                    armLiftMotor.setPower(0);
                    armLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }


            // Shift the motor power up/down by how much the triggers have been pushed down since the last loop
            // Math.max() picks the greater of two numbers, so negative movements of the trigger (trigger is pulled in) will become 0.0
            // This way, the motor power will only be affected as the triggers are pressed down and not when they are released
            float upShift = Math.max(0.0f, rTNow - rTLast) * 0.05f;
            float downShift = Math.max(0.0f, lTNow - lTLast) * 0.05f;
            if (controlMode == 0) {
                driveSubsystem.setPowerMultiplier(Math.max(Math.min(driveSubsystem.getPowerMultiplier() + upShift - downShift, 1), 0));
            } else if (controlMode == 1) {
                armStendoPower = Math.max(Math.min(armStendoPower + upShift - downShift, 1), 0);
            } else {
                armLiftPower = Math.max(Math.min(armLiftPower + upShift - downShift, 1), 0);
            }

            // Run motors
            if (controlMode == 3) {
                // Control sticks control arm
                double rightStickMagnitude = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
                double leftStickMagnitude = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
                if (rightStickMagnitude > Constants.ARM_CONTROL_STICK_THRESHOLD) {
                    int armInputTarget = Calculations.vectorToArmPositionHalf(gamepad1.right_stick_x, gamepad1.right_stick_y);
                    armLiftMotor.setTargetPosition(armInputTarget);
                } else if (leftStickMagnitude > Constants.ARM_CONTROL_STICK_THRESHOLD) {
                    int armInputTarget = Calculations.vectorToArmPositionFull(gamepad1.left_stick_x, gamepad1.left_stick_y);
                    armLiftMotor.setTargetPosition(armInputTarget);
                }
            } else if (controlMode < 3) {
                // Control sticks control drive
                QuadMotorValues<Double> drivePower = Calculations.mecanumDriveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                driveSubsystem.setPower(drivePower);
            }


            // Add info to be shown on the driver control station
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("", "");
            telemetry.addData("Control mode:", controlMode);
            telemetry.addData("Limp motors?", isLimp);
            telemetry.addData("", "");
            telemetry.addData("Left  claw servo position:", leftClawServo.getPosition());
            telemetry.addData("Right claw servo position:", rightClawServo.getPosition());
            telemetry.addData("", "");
            telemetry.addData("Wrist scaled position:", Calculations.encoderToScaleArmWrist(armWristServo.getPosition()));
            telemetry.addData("Wrist servo position:", armWristServo.getPosition());
            telemetry.addData("", "");
            telemetry.addData("Arm stendo power level:", (armStendoPower * 100.0) + "%");
            telemetry.addData("Arm stendo scaled position:", Calculations.encoderToScaleArmExtension(armTravelMotor.getCurrentPosition()));
            telemetry.addData("Arm stendo encoder position:", armTravelMotor.getCurrentPosition());
            telemetry.addData("Arm stendo motor power:", armTravelMotor.getPower());
            telemetry.addData("", "");
            telemetry.addData("Arm lift power level:", (armLiftPower * 100.0) + "%");
            telemetry.addData("Arm lift scaled position:", Calculations.encoderToScaleArmRotation(armLiftMotor.getCurrentPosition()));
            telemetry.addData("Arm lift encoder position:", armLiftMotor.getCurrentPosition());
            if (controlMode == 3) {
                telemetry.addData("Arm lift target position:", armLiftMotor.getTargetPosition());
            } else {
                telemetry.addData("Arm lift motor power:", armLiftMotor.getPower());
            }
            telemetry.addData("", "");
            telemetry.addData("Drive motor power level:", (driveSubsystem.getPowerMultiplier() * 100.0) + "%");
            telemetry.addData("Front left/right position in inches:", "%.2f, %.2f", Calculations.encoderToInchesDrive(frontLeftDrive.getCurrentPosition()), Calculations.encoderToInchesDrive(frontRightDrive.getCurrentPosition()));
            telemetry.addData("Back  left/right position in inches:", "%.2f, %.2f", Calculations.encoderToInchesDrive(backLeftDrive.getCurrentPosition()), Calculations.encoderToInchesDrive(backRightDrive.getCurrentPosition()));
            telemetry.addData("Front left/right encoder position:", "%d, %d", frontLeftDrive.getCurrentPosition(), frontRightDrive.getCurrentPosition());
            telemetry.addData("Back  left/right encoder position:", "%d, %d", backLeftDrive.getCurrentPosition(), backRightDrive.getCurrentPosition());
            telemetry.addData("Front left/right motor power:", "%d%%, %d%%", Math.round(frontLeftDrive.getPower() * 100), Math.round(frontRightDrive.getPower() * 100));
            telemetry.addData("Back  left/right motor power:", "%d%%, %d%%", Math.round(backLeftDrive.getPower() * 100), Math.round(backRightDrive.getPower() * 100));
            telemetry.update();

            // Update previous inputs
            rTLast = rTNow;
            lTLast = lTNow;
            startLast = startNow;
            guideLast = guideNow;
        }
    }
}
