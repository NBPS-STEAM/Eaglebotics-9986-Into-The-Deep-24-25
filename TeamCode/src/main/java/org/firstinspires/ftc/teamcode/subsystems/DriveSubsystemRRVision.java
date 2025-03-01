package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.*;
import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Calculations;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.helper.DriverPrompter;
import org.firstinspires.ftc.teamcode.helper.ResetZeroState;
import org.firstinspires.ftc.teamcode.helper.QuadMotorValues;
import org.firstinspires.ftc.teamcode.helper.RoadRunnerCommand;
import org.firstinspires.ftc.teamcode.helper.localization.DriveLocalizer;
import org.firstinspires.ftc.teamcode.helper.localization.Localizers;
import org.firstinspires.ftc.teamcode.helper.localization.VisionDriveLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.roadrunner.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;

import java.lang.Math;
import java.util.*;

/**
 * This is a class for a mecanum drivetrain using Road Runner.
 * <p>This is tuned for when the raise is fully extended, and it drives at a somewhat aggressive speed.</p>
 * <p>This version of the tune uses VisionPortalSubsystem for localization with AprilTags!</p>
 * <p>Please see {@link MecanumDrive} for more information.</p>
 */
public class DriveSubsystemRRVision extends SubsystemBase implements VisionPortalSubsystem.GyroSource {
    public static class Params {
        // IMU orientation
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = Constants.IMU_HUB_LOGO_DIRECTION;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = Constants.IMU_HUB_USB_DIRECTION;

        // drive model parameters
        public double inPerTick = 122.25 / 4071.75;
        public double lateralInPerTick = 125.75 / 3981.25;
        public double trackWidthTicks = 815.794386053208;

        // feedforward parameters (in tick units)
        public double kS = 2.0124767328697315;
        public double kV = 0.0038539743279538442;
        public double kA = 0.00005;

        // path profile parameters (in inches)
        public double maxWheelVel = Constants.AUTO_DRIVE_VEL_MAX;
        public double minProfileAccel = -Constants.AUTO_DRIVE_ACCEL_MAX;
        public Double maxProfileAccel = Constants.AUTO_DRIVE_ACCEL_MAX;

        // turn profile parameters (in radians)
        public double maxAngVel = Constants.AUTO_DRIVE_ANG_VEL_MAX; // shared with path
        public double maxAngAccel = Constants.AUTO_DRIVE_ANG_ACCEL_MAX;

        // path controller gains
        public double axialGain = 3.0;
        public double lateralGain = 2.5;
        public double headingGain = 2.0; // shared with turn

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // shared with turn
    }

    public final Params PARAMS;

    public final MecanumKinematics kinematics;

    public final TurnConstraints defaultTurnConstraints;
    public final VelConstraint defaultVelConstraint;
    public final AccelConstraint defaultAccelConstraint;

    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public final VoltageSensor voltageSensor;

    public final LazyImu lazyImu;

    public final Localizer localizer;
    public final VisionPortalSubsystem vps;
    public Pose2d pose;
    public Boolean flashVision = null; // true to signal it to use the next vision scan; null for continuous vision

    private final KalmanFilter positionFilterX = new KalmanFilter(Constants.ABS_LOCALIZER_DENOISE_Q, Constants.ABS_LOCALIZER_DENOISE_R, Constants.ABS_LOCALIZER_DENOISE_N);
    private final KalmanFilter positionFilterY = new KalmanFilter(Constants.ABS_LOCALIZER_DENOISE_Q, Constants.ABS_LOCALIZER_DENOISE_R, Constants.ABS_LOCALIZER_DENOISE_N);

    private boolean isBlueAlliance = false;
    private boolean isAllianceTrusted = false;
    private boolean wasAllianceChanged = false;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    /**
     * Create a DriveSubsystemRRVision with default autonomous driving parameters.
     * <p>Make sure to use {@link #setIsBlueAlliance(boolean, boolean)} for alliance-aware features to use the right alliance
     * (default: red alliance)</p>
     * @see #setIsBlueAlliance(boolean, boolean)
     */
    public DriveSubsystemRRVision(HardwareMap hardwareMap, VisionPortalSubsystem visionPortalSubsystem, Localizers localizer, Pose2d pose) {
        this(hardwareMap, visionPortalSubsystem, localizer, pose, new Params());
    }

    /**
     * Create a DriveSubsystemRRVision with custom autonomous driving parameters.
     * <p>If vision is not used, then visionPortalSubsystem may be left null.</p>
     * <p>Make sure to use {@link #setIsBlueAlliance(boolean, boolean)} for alliance-aware features to use the right alliance
     * (default: red alliance)</p>
     * @see #setIsBlueAlliance(boolean, boolean)
     */
    public DriveSubsystemRRVision(HardwareMap hardwareMap, VisionPortalSubsystem visionPortalSubsystem, Localizers localizer, Pose2d pose, Params parameters) {
        // Parameters
        PARAMS = parameters;
        vps = visionPortalSubsystem;

        kinematics = new MecanumKinematics(
                PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

        defaultTurnConstraints = new TurnConstraints(
                PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
        defaultVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                        new AngularVelConstraint(PARAMS.maxAngVel)
                ));
        defaultAccelConstraint =
                new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

        // Initialize

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFront = hardwareMap.get(DcMotorEx.class, Constants.NAME_DRIVE_FL);
        leftBack = hardwareMap.get(DcMotorEx.class, Constants.NAME_DRIVE_BL);
        rightBack = hardwareMap.get(DcMotorEx.class, Constants.NAME_DRIVE_BR);
        rightFront = hardwareMap.get(DcMotorEx.class, Constants.NAME_DRIVE_FR);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(Constants.DIRECTION_DRIVE_FL);
        leftBack.setDirection(Constants.DIRECTION_DRIVE_BL);
        rightBack.setDirection(Constants.DIRECTION_DRIVE_BR);
        rightFront.setDirection(Constants.DIRECTION_DRIVE_FR);

        lazyImu = new LazyImu(hardwareMap, Constants.NAME_IMU, new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));
        lazyImu.get().resetYaw();

        // Preserve previous heading
        Vector2d newPosition = pose.position;
        if (!ResetZeroState.shouldZeroDrive() && ResetZeroState.getPrevDrivePos() != null) newPosition = ResetZeroState.getPrevDrivePos();

        double newHeading = pose.heading.toDouble();
        if (ResetZeroState.shouldZeroHeading()) {
            zeroDriverHeading();
        } else if (ResetZeroState.getPrevHeading() != null) {
            newHeading = ResetZeroState.getPrevHeading();
            zeroDriverHeading(newHeading);
        }

        this.pose = new Pose2d(newPosition, newHeading);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        switch (localizer) {
            case ENCODERS_ONLY:
                this.localizer = new DriveLocalizer(this);
                break;
            case ENCODERS_WITH_VISION:
                if (Constants.USE_GYRO_ESTIMATOR) vps.enableGyroLocalizer(this);
                this.localizer = new VisionDriveLocalizer(this, vps);
                break;
            default:
                throw new IllegalArgumentException("Unsupported localizer chosen: " + localizer);
        }

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    /**
     * Set the alliance to use for localization, etc.
     * <p>If {@code trusted} is false, then the alliance may be automatically reobtained later.</p>
     */
    public void setIsBlueAlliance(boolean isBlueAlliance, boolean trusted) {
        DriverPrompter.setOnBlueAlliance(isBlueAlliance);
        this.isBlueAlliance = isBlueAlliance;
        isAllianceTrusted = trusted;
        localizer.setAlliance(isBlueAlliance);
    }

    public boolean getIsBlueAlliance() {
        return isBlueAlliance;
    }

    // DRIVER CONTROLS

    public boolean isFieldCentric = true;
    public double setPowerMultiplier = Constants.DRIVE_POWER_MULTIPLIER;

    private double driverHeadingOffset = 0.0;

    public void zeroDriverHeading() {
        zeroDriverHeading(0.0);
    }
    public void zeroDriverHeading(double atAngle) {
        driverHeadingOffset = atAngle - getImuHeading();
    }

    public double getImuHeading() {
        return lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getDriverHeading() {
        return getImuHeading() + driverHeadingOffset;
    }

    @Override
    public double getFieldHeading() {
        return getDriverHeading() + (isBlueAlliance ? -Math.PI / 2 : Math.PI / 2);
    }

    public void setSetPowerMultiplier(double mult) {
        setPowerMultiplier = mult;
    }

    public void setFullSpeed() {
        setSetPowerMultiplier(Constants.DRIVE_POWER_MULTIPLIER);
    }

    public void setMediumSpeed() {
        setSetPowerMultiplier(Constants.DRIVE_POWER_MULTIPLIER_MED);
    }

    public void setSlowSpeed() {
        setSetPowerMultiplier(Constants.DRIVE_POWER_MULTIPLIER_SLOW);
    }

    public void drive(GamepadEx gamepad) {
        drive(gamepad.getLeftY(), gamepad.getLeftX(), gamepad.getRightX());
    }
    public void drive(Gamepad gamepad) {
        drive(-gamepad.left_stick_y, -gamepad.left_stick_x, gamepad.right_stick_x);
    }
    public void drive(double axial, double lateral, double yaw) {
        axial *= setPowerMultiplier;
        lateral *= setPowerMultiplier;
        yaw *= setPowerMultiplier;
        QuadMotorValues<Double> drivePower;
        if (isFieldCentric) {
            drivePower = Calculations.mecanumDriveFieldCentric(axial, lateral, yaw, getDriverHeading());
        } else {
            drivePower = Calculations.mecanumDriveRobotCentric(axial, lateral, yaw);
        }
        setDrivePowers(drivePower);

        updatePoseEstimate();
    }

    public void setDrivePowers(QuadMotorValues<Double> power) {
        leftFront.setPower(power.getFrontLeftValue());
        rightFront.setPower(power.getFrontRightValue());
        leftBack.setPower(power.getBackLeftValue());
        rightBack.setPower(power.getBackRightValue());
    }

    // END OF DRIVER CONTROLS

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            Pose2d error = txWorldTarget.value().minusExp(pose);

            if ((t >= timeTrajectory.duration
                    && error.position.norm() < Constants.AUTO_DRIVE_POS_TOLERANCE
                    && robotVelRobot.linearVel.norm() < Constants.AUTO_DRIVE_VEL_TOLERANCE
                    && error.heading.toDouble() < Constants.AUTO_DRIVE_ANG_TOLERANCE
                    && robotVelRobot.angVel < Constants.AUTO_DRIVE_ANGVEL_TOLERANCE)
                    || t >= timeTrajectory.duration + Constants.AUTO_DRIVE_TOLERANCE_TIMEOUT) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            //Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            //targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            //PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            rightFront.setPower(rightFrontPower);

            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            //Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            Pose2d error = txWorldTarget.value().minusExp(pose);

            if ((t >= turn.duration
                    && error.heading.toDouble() < Constants.AUTO_DRIVE_ANG_TOLERANCE
                    && robotVelRobot.angVel < Constants.AUTO_DRIVE_ANGVEL_TOLERANCE)
                    || t >= turn.duration + Constants.AUTO_DRIVE_TOLERANCE_TIMEOUT) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            //Pose2dDual<Time> txWorldTarget = turn.get(t);
            //targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            //PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public boolean didLastPoseEstUseVision() {
        return usedVision;
    }
    private boolean usedVision = false;

    /*private boolean isTwistUnderLimit(Twist2dDual<Time> twist) {
        return (twist.velocity().value().linearVel.sqrNorm() < Constants.LOCALIZATION_LINEAR_THRESHOLD*Constants.LOCALIZATION_LINEAR_THRESHOLD)
                && (Math.abs(twist.velocity().value().angVel) < Constants.LOCALIZATION_ANGULAR_THRESHOLD);
    }*/

    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();

        Pose2d absPose = null;
        if (/*isTwistUnderLimit(twist) && */(flashVision == null || flashVision)) {
            absPose = localizer.getAbsolutePosition();

            if (!isAllianceTrusted) {
                if (localizer.getLastUpdateStatus() == Localizers.Status.DUBIOUS_ALLIANCE) {
                    setIsBlueAlliance(!isBlueAlliance, false);
                    absPose = localizer.calculateAbsolutePosition();
                }
            }
        }

        usedVision = absPose != null;
        if (usedVision && flashVision != null) flashVision = false;

        if (absPose == null) {
            // No absolute pose available (i.e. no vision)
            pose = pose.plus(twist.value());
            if (Constants.ABS_LOCALIZER_DENOISE) {
                // The position filter updates continuously, so update it even though it's not used here
                positionFilterX.estimate(pose.position.x);
                positionFilterY.estimate(pose.position.y);
            }
            pose = new Pose2d(pose.position, getDriverHeading() + Math.PI / 2); // gyro hack
        } else {
            // Absolute pose is available (i.e. vision success)
            double posX = absPose.position.x;
            double posY = absPose.position.y;
            if (Constants.ABS_LOCALIZER_DENOISE) {
                posX = positionFilterX.estimate(posX);
                posY = positionFilterY.estimate(posY);
            }
            pose = new Pose2d(new Vector2d(posX, posY), absPose.heading);
        }

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));

        return twist.velocity().value();
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    public void addPoseTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addLine("ROBOT POSE:");
        telemetry.addData("Position X", pose.position.x);
        telemetry.addData("Position Y", pose.position.y);
        telemetry.addData("Heading", pose.heading.toDouble());
        telemetry.addData("Heading (gyro)", getDriverHeading());
    }

    public class TelemetryLoggerCommand extends CommandBase {
        private final Telemetry telemetry;

        public TelemetryLoggerCommand(Telemetry telemetry) {
            this.telemetry = telemetry;
        }

        @Override
        public void initialize() {
            telemetry.setAutoClear(true);
        }

        @Override
        public void execute() {
            addPoseTelemetry(telemetry);
            telemetry.update();
        }

        @Override
        public void end(boolean interrupted) {
            telemetry.clear();
        }
    }
}
