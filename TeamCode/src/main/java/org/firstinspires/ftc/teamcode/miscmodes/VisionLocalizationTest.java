package org.firstinspires.ftc.teamcode.miscmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.helper.DriverPrompter;
import org.firstinspires.ftc.teamcode.helper.localization.Localizers;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystemRRVision;
import org.firstinspires.ftc.teamcode.subsystems.VisionPortalSubsystem;

@TeleOp(name = "Vision Localization Testing", group = "Vision Config")
public class VisionLocalizationTest extends LinearOpMode {
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
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("tag visible?", drive.didLastPoseEstUseVision() ? 10 : 0);
            telemetry.addLine();
            telemetry.addLine("Set alliance for localization:");
            telemetry.addLine("Press Left bumper - Blue Alliance");
            telemetry.addLine("Press Right bumper - Red Alliance");
            if (gamepad1.left_bumper || gamepad1.right_bumper) {
                drive.setIsBlueAlliance(gamepad1.left_bumper);
            }
            telemetry.addData("Alliance", drive.getIsBlueAlliance() ? "Blue Alliance" : "Red Alliance");
            telemetry.addLine();
            telemetry.addLine("Set vision localization method:");
            telemetry.addLine("Press Select - Default Vision-Only");
            telemetry.addLine("Press Start - Custom Gyro-Vision");
            if (gamepad1.back) {
                vps.enableGyroLocalizer(null);
            } else if (gamepad1.start) {
                vps.enableGyroLocalizer(drive, telemetry);
            }
            telemetry.addData("Vision localization method", vps.getGyroLocalizer() == null ? "Vision-Only" : "Gyro-Vision");
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
