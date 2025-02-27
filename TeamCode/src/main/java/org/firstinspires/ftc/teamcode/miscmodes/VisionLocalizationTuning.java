package org.firstinspires.ftc.teamcode.miscmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.helper.localization.Localizers;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystemRRVision;
import org.firstinspires.ftc.teamcode.subsystems.VisionPortalSubsystem;

@TeleOp(name = "Vision Localization Tuning", group = "Vision Config")
public class VisionLocalizationTuning extends LinearOpMode {
    private VisionPortalSubsystem vps;
    private DriveSubsystemRRVision drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        vps = new VisionPortalSubsystem(hardwareMap, true);
        drive = new DriveSubsystemRRVision(hardwareMap, vps,
                Localizers.ENCODERS_WITH_VISION, new Pose2d(0, 0, 0));

        drive.setMediumSpeed();

        waitForStart();

        while (opModeIsActive()) {
            drive.drive(gamepad1);

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addLine();
            if (vps.isVisionPortalStreaming()) {
                if (gamepad1.start) {
                    vps.setExposureMode(vps.getExposureMode() == ExposureControl.Mode.Manual ? ExposureControl.Mode.Auto : ExposureControl.Mode.Manual);
                }
                if (gamepad1.left_bumper) {
                    changeExposure(1);
                }
                if (gamepad1.left_trigger > 0.8) {
                    changeExposure(-1);
                }
                if (gamepad1.right_bumper) {
                    changeGain(1);
                }
                if (gamepad1.right_trigger > 0.8) {
                    changeGain(-1);
                }

                telemetry.addLine("Find lowest Exposure that gives reliable detection.");
                telemetry.addLine("Use Left bump/trig to adjust Exposure.");
                telemetry.addLine("Use Right bump/trig to adjust Gain.");
                telemetry.addLine("Use Start button to toggle auto/manual exposure.");
                telemetry.addLine();
                telemetry.addData("TAGS DETECTED", vps.getDetections().size());
                telemetry.addData("Exposure Mode", vps.getExposureMode());
                telemetry.addData("Exposure", "%d  (%d - %d)", vps.getExposure(), vps.getMinExposure(), vps.getMaxExposure());
                telemetry.addData("Gain", "%d  (%d - %d)", vps.getGain(), vps.getMinGain(), vps.getMaxGain());
            } else {
                telemetry.addLine("Waiting on camera...");
            }
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    private void changeExposure(long delta) {
        // For some reason, Range.clip() doesn't support longs. Weird.
        vps.setExposure(Math.min(Math.max(vps.getExposure() + delta, vps.getMinExposure()), vps.getMaxExposure()));
    }

    private void changeGain(int delta) {
        vps.setGain(Range.clip(vps.getGain() + delta, vps.getMinGain(), vps.getMaxGain()));
    }
}
