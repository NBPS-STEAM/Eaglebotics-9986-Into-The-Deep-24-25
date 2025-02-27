package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
        NormalizedRGBA[] sensorColors = armSubsystem.getColorReadings();

        // Add info to be shown on the Driver Station
        telemetry.addData("Status", "Run Time: " + mainOpMode.getRuntime());
        telemetry.addLine();
        telemetry.addData("IMU heading direction (degrees)", driveSubsystem.getHeading(AngleUnit.DEGREES));
        telemetry.addLine();
        telemetry.addData("Intake sensor distance (mm)", armSubsystem.getRangeReadingMM());
        for (int i = 0; i < sensorColors.length; i++) {
            telemetry.addData("Intake sensor " + (i + 1) + " color", colorToString(sensorColors[i]));
        }
        telemetry.addLine();
        telemetry.addData("Intake servos power", armSubsystem.getIntakePower());
        telemetry.addLine();
        telemetry.addData("Wrist scaled position", armSubsystem.getWristPosition());
        telemetry.addData("Wrist servo position", armSubsystem.getWristPositionUnscaled());
        telemetry.addLine();
        telemetry.addData("Arm extension power level", (Constants.ARM_EXTENSION_POWER * 100.0) + "%");
        telemetry.addData("Arm extension scaled position", armSubsystem.getExtensionPosition());
        telemetry.addData("Arm extension encoder position", armSubsystem.getExtensionPositionUnscaled());
        telemetry.addData("Arm extension scaled target position", armSubsystem.getExtensionTargetPosition());
        telemetry.addData("Arm extension encoder target position", armSubsystem.getExtensionTargetPositionUnscaled());
        telemetry.addLine();
        telemetry.addData("Arm raise power level", (Constants.ARM_RAISE_POWER * 100.0) + "%");
        telemetry.addData("Arm raise scaled position", armSubsystem.getRaisePosition());
        telemetry.addData("Arm raise encoder position", armSubsystem.getRaisePositionUnscaled());
        telemetry.addData("Arm raise scaled target position", armSubsystem.getRaiseTargetPosition());
        telemetry.addData("Arm raise encoder target position", armSubsystem.getRaiseTargetPositionUnscaled());
        telemetry.addLine();
        telemetry.addData("Arm rotation power level", (Constants.ARM_ROTATION_POWER * 100.0) + "%");
        telemetry.addData("Arm rotation scaled position", armSubsystem.getRotationPosition());
        telemetry.addData("Arm rotation encoder position", armSubsystem.getRotationPositionUnscaled());
        telemetry.addData("Arm rotation scaled target position", armSubsystem.getRotationTargetPosition());
        telemetry.addData("Arm rotation encoder target position", armSubsystem.getRotationTargetPositionUnscaled());
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addData("Drive motor power level:", (driveSubsystem.getPowerMultiplier() * 100.0) + "%");
        telemetry.addData("Front left/right motor positions (in)", "%.2f, %.2f", drivePositions.getFrontLeftValue(), drivePositions.getFrontRightValue());
        telemetry.addData("Back  left/right motor positions (in)", "%.2f, %.2f", drivePositions.getBackLeftValue(), drivePositions.getBackRightValue());
        telemetry.addData("Front left/right motor encoder positions", "%d, %d", drivePositionsUnscaled.getFrontLeftValue(), drivePositionsUnscaled.getFrontRightValue());
        telemetry.addData("Back  left/right motor encoder positions", "%d, %d", drivePositionsUnscaled.getBackLeftValue(), drivePositionsUnscaled.getBackRightValue());
        telemetry.addData("Front left/right motor powers", "%d%%, %d%%", Math.round(drivePowers.getFrontLeftValue() * 100.0), Math.round(drivePowers.getFrontRightValue() * 100.0));
        telemetry.addData("Back  left/right motor powers", "%d%%, %d%%", Math.round(drivePowers.getBackLeftValue() * 100.0), Math.round(drivePowers.getBackRightValue() * 100.0));

        // Send info to the Driver Station
        telemetry.update();
    }

    public static String colorToString(NormalizedRGBA color) {
        return "r: " + color.red + " g: " + color.green + " b: " + color.blue;
    }
}
