package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Calculations;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.helper.QuadMotorValues;

/**
 * This is a class for a "subsystem" which reports the status of a DriveSubsystem and an ArmSubsystem.
 */
public class TelemetrySubsystem extends SubsystemBase {

    // Private instance variables
    private final LinearOpMode mainOpMode;
    private final Telemetry telemetry;
    private final DriveSubsystem driveSubsystem;
    private final ArmSubsystem armSubsystem;

    private boolean enabled = true;


    // Constructor methods
    public TelemetrySubsystem(LinearOpMode mainOpMode, DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem) {
        this.mainOpMode = mainOpMode;
        this.driveSubsystem = driveSubsystem;
        this.armSubsystem = armSubsystem;

        this.telemetry = mainOpMode.telemetry;
    }


    // Methods

    // Enabled/disabled

    public void enableTelemetry() {
        enabled = true;
    }

    public void disableTelemetry() {
        enabled = true;
    }

    public void toggleTelemetry() {
        enabled = !enabled;
    }

    public boolean getEnabled() {
        return enabled;
    }

    // Telemetry Reporting

    /**
     * If this TelemetrySubsystem is disabled, this does nothing.
     */
    public void reportTelemetry() {
        // End immediately if disabled
        if (!enabled) return;

        // Get some info
        QuadMotorValues<Double> drivePositions = driveSubsystem.getMotorPositions();
        QuadMotorValues<Integer> drivePositionsUnscaled = driveSubsystem.getMotorPositionsUnscaled();
        QuadMotorValues<Double> drivePowers = driveSubsystem.getMotorPowers();

        // Clear old info from the Driver Station
        telemetry.clear();

        // Add info to be shown on the Driver Station
        telemetry.addData("Status", "Run Time: " + mainOpMode.getRuntime());
        telemetry.addData("", "");
        telemetry.addData("Intake servo power:", armSubsystem.getIntakePower());
        telemetry.addData("", "");
        telemetry.addData("Wrist scaled position:", armSubsystem.getWristPosition());
        telemetry.addData("Wrist servo position:", armSubsystem.getWristPositionUnscaled());
        telemetry.addData("", "");
        telemetry.addData("Arm extension power level:", (Constants.ARM_EXTENSION_POWER * 100.0) + "%");
        telemetry.addData("Arm extension scaled position:", armSubsystem.getExtensionPosition());
        telemetry.addData("Arm extension encoder position:", armSubsystem.getExtensionPositionUnscaled());
        telemetry.addData("Is arm extension at target position?", armSubsystem.getExtensionMotor().atTargetPosition());
        telemetry.addData("", "");
        telemetry.addData("Arm raise power level:", (Constants.ARM_RAISE_POWER * 100.0) + "%");
        telemetry.addData("Arm raise scaled position:", armSubsystem.getRaisePosition());
        telemetry.addData("Arm raise encoder position:", armSubsystem.getRaisePositionUnscaled());
        telemetry.addData("Is arm raise at target position?", armSubsystem.getRaiseMotor().atTargetPosition());
        telemetry.addData("", "");
        telemetry.addData("Arm rotation power level:", (Constants.ARM_ROTATION_POWER * 100.0) + "%");
        telemetry.addData("Arm rotation scaled position:", armSubsystem.getRotationPosition());
        telemetry.addData("Arm rotation encoder position:", armSubsystem.getRotationPositionUnscaled());
        telemetry.addData("Is arm rotation at target position?", armSubsystem.getRotationMotor().atTargetPosition());
        telemetry.addData("", "");
        telemetry.addData("", "");
        telemetry.addData("Drive motor power level:", (driveSubsystem.getPowerMultiplier() * 100.0) + "%");
        telemetry.addData("Front left/right position in inches:", "%.2f, %.2f", drivePositions.getFrontLeftValue(), drivePositions.getFrontRightValue());
        telemetry.addData("Back  left/right position in inches:", "%.2f, %.2f", drivePositions.getBackLeftValue(), drivePositions.getBackRightValue());
        telemetry.addData("Front left/right encoder position:", "%d, %d", drivePositionsUnscaled.getFrontLeftValue(), drivePositionsUnscaled.getFrontRightValue());
        telemetry.addData("Back  left/right encoder position:", "%d, %d", drivePositionsUnscaled.getBackLeftValue(), drivePositionsUnscaled.getBackRightValue());
        telemetry.addData("Front left/right motor power:", "%d%%, %d%%", Math.round(drivePowers.getFrontLeftValue() * 100), Math.round(drivePowers.getFrontRightValue() * 100));
        telemetry.addData("Back  left/right motor power:", "%d%%, %d%%", Math.round(drivePowers.getBackLeftValue() * 100), Math.round(drivePowers.getBackRightValue() * 100));

        // Send info to the Driver Station
        telemetry.update();
    }
}
