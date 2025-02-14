package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;
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
public class DriveSubsystemRRVision extends SubsystemBase {
    public static class Params {
        // IMU orientation
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = Constants.IMU_HUB_LOGO_DIRECTION;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = Constants.IMU_HUB_USB_DIRECTION;

        // drive model parameters
        public double inPerTick = 121.75 / 4032.75;
        public double lateralInPerTick = 125.75 / 3981.25;
        public double trackWidthTicks = 815.794386053208;

        // feedforward parameters (in tick units)
        public double kS = 2.0124767328697315;
        public double kV = 0.0038539743279538442;
        public double kA = 0.00005;

        // path profile parameters (in inches)
        public double maxWheelVel = 50.0;
        public double minProfileAccel = -30.0;
        public Double maxProfileAccel = 50.0;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI * 1.2; // shared with path
        public double maxAngAccel = Math.PI * 1.2;

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
    public Pose2d pose; // TODO: better way to preserve heading between auto and teleop

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    /**
     * Create a DriveSubsystemRRVision with default autonomous driving parameters.
     * <p>Make sure to use {@link #setIsBlueAlliance(boolean)} for alliance-aware features to use the right alliance
     * (default: red alliance)</p>
     * @see #setIsBlueAlliance(boolean)
     */
    public DriveSubsystemRRVision(HardwareMap hardwareMap, VisionPortalSubsystem visionPortalSubsystem, Localizers localizer, Pose2d pose) {
        this(hardwareMap, visionPortalSubsystem, localizer, pose, new Params());
    }

    /**
     * Create a DriveSubsystemRRVision with custom autonomous driving parameters.
     * <p>If vision is not used, then visionPortalSubsystem may be left null.</p>
     * <p>Make sure to use {@link #setIsBlueAlliance(boolean)} for alliance-aware features to use the right alliance
     * (default: red alliance)</p>
     * @see #setIsBlueAlliance(boolean)
     */
    public DriveSubsystemRRVision(HardwareMap hardwareMap, VisionPortalSubsystem visionPortalSubsystem, Localizers localizer, Pose2d pose, Params parameters) {
        // Parameters
        PARAMS = parameters;

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
        Vector2d newPosition = DriveSubsystem.zeroDriveOnInit ? pose.position : this.pose.position;
        Rotation2d newHeading = DriveSubsystem.zeroHeadingOnInit ? pose.heading : this.pose.heading;
        this.pose = new Pose2d(newPosition, newHeading);
        DriveSubsystem.zeroDriveOnInit = true;
        DriveSubsystem.zeroHeadingOnInit = true;

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

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        switch (localizer) {
            case ENCODERS_ONLY:
                this.localizer = new DriveLocalizer(this);
                break;
            case ENCODERS_WITH_VISION:
                this.localizer = new VisionDriveLocalizer(this, visionPortalSubsystem);
                break;
            default:
                throw new IllegalArgumentException("Unsupported localizer chosen: " + localizer);
        }

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);

        // TODO: better way to preserve heading between auto and teleop
        zeroDriverHeading(this.pose.heading.toDouble());
    }

    public void setIsBlueAlliance(boolean isBlueAlliance) {
        localizer.setAlliance(isBlueAlliance);
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
        leftFront.setPower(power.getFrontLeftValue() * setPowerMultiplier);
        rightFront.setPower(power.getFrontRightValue() * setPowerMultiplier);
        leftBack.setPower(power.getBackLeftValue() * setPowerMultiplier);
        rightBack.setPower(power.getBackRightValue() * setPowerMultiplier);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag * setPowerMultiplier);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag * setPowerMultiplier);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag * setPowerMultiplier);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag * setPowerMultiplier);
    }

    /**
     * This command wraps a Road Runner command to autonomously drive and score in the high basket before
     * returning to the submersible zone.
     * <p>I HIGHLY RECOMMEND that if this is used in teleop, such is done immediately after localizing with vision.
     * Otherwise, the localization will almost certainly be off if it's relying only on encoders.</p>
     */
    public Command getDriveToBasketCommand(ArmSubsystem armSubsystem) {
        Action[] actions = getDriveToBasketActions(armSubsystem);
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(() -> isArmSubsystemAtTarget(armSubsystem)),
                        new RoadRunnerCommand(actions[0])
                ),
                new RoadRunnerCommand(actions[1])
        );
        //return new DriveToBasketCommand(armSubsystem);
    }

    private boolean isArmSubsystemAtTarget(ArmSubsystem armSubsystem) {
        return armSubsystem.isRotationAtTargetPosition() && armSubsystem.isRaiseAtTargetPosition();
    }

    private Action[] getDriveToBasketActions(ArmSubsystem armSubsystem) {
        Pose2d approachPos = Constants.POS_BASKETS_APPROACH;
        Pose2d scorePos = Constants.POS_BASKETS_SCORE;
        Pose2d intakePos = Constants.POS_INTAKE_APPROACH;
        return new Action[] {
                // First path (approach basket)
                actionBuilder(intakePos)
                        .stopAndAdd(() -> armSubsystem.lockSetPosition(false))
                        .stopAndAdd(() -> armSubsystem.applyNamedPosition("basket high", true, true))
                        .strafeToLinearHeading(approachPos.position, approachPos.heading)
                        .build(),

                // Second path (score in basket)
                actionBuilder(approachPos)
                        .strafeToLinearHeading(scorePos.position, scorePos.heading)
                        .stopAndAdd(armSubsystem::startOuttake)
                        .waitSeconds(0.3)
                        .afterTime(0.5, () -> {
                            armSubsystem.lockSetPosition(false);
                            armSubsystem.applyNamedPosition("stow");
                        })
                        .strafeToLinearHeading(intakePos.position, intakePos.heading)
                        .build()
        };
    }

    /*class DriveToBasketCommand implements Command {
        @Override
        public Set<Subsystem> getRequirements() {
            return Collections.singleton(DriveSubsystemRRVision.this);
        }

        public DriveToBasketCommand(ArmSubsystem armSubsystem) {
            this.armSubsystem = armSubsystem;
        }

        // Configuration variables
        private final ArmSubsystem armSubsystem;
        private final TelemetryPacket packet = new TelemetryPacket();

        // Running variables
        private Pose2d approachPos;
        private Pose2d scorePos;
        private Pose2d intakePos;
        private Action path1;
        private boolean runAgain1;
        private Action path2;
        private boolean runAgain2;

        @Override
        public void initialize() {
            if (isBlueAlliance) {
                approachPos = Constants.POS_BASKETS_APPROACH_BLUE;
                scorePos = Constants.POS_BASKETS_SCORE_BLUE;
                intakePos = Constants.POS_INTAKE_APPROACH_BLUE;
            } else {
                approachPos = Constants.POS_BASKETS_APPROACH_RED;
                scorePos = Constants.POS_BASKETS_SCORE_RED;
                intakePos = Constants.POS_INTAKE_APPROACH_RED;
            }
            path1 = null;
            runAgain1 = true;
            path2 = null;
            runAgain2 = true;
        }

        @Override
        public void execute() {
            // This command will run the 'first' path to approach the baskets while the arm raises,
            // and then abruptly switch to the 'second' path to score as soon as the arm is ready.
            if (path1 == null) { // If the first path hasn't started yet...
                // Compose and start the first path
                path1 = actionBuilder(intakePos)
                        .strafeToLinearHeading(approachPos.position, approachPos.heading)
                        .build();

                armSubsystem.lockSetPosition(false);
                armSubsystem.applyNamedPosition("basket high", true, true);
                runAgain1 = path1.run(packet);
            } else if (path2 == null) { // If the second path hasn't started yet...
                if (!armSubsystem.isRotationAtTargetPosition() || !armSubsystem.isRaiseAtTargetPosition()) {
                    if (runAgain1) {
                        runAgain1 = path1.run(packet);
                    }
                } else { // If both motors are at their target positions...
                    // Compose and start the second path
                    path2 = actionBuilder(approachPos)
                            .strafeToLinearHeading(scorePos.position, scorePos.heading)
                            .stopAndAdd(armSubsystem::startOuttake)
                            .waitSeconds(0.3)
                            .afterTime(0.5, () -> {
                                armSubsystem.lockSetPosition(false);
                                armSubsystem.applyNamedPosition("stow");
                            })
                            .strafeToLinearHeading(intakePos.position, intakePos.heading)
                            .build();
                    runAgain2 = path2.run(packet);
                }
            } else if (runAgain2) { // If the second path is still running...
                runAgain2 = path2.run(packet);
            }
        }

        @Override
        public boolean isFinished() {
            // End when the second path finishes.
            return !runAgain2;
        }
    }*/

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

            if (t >= timeTrajectory.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

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

            Pose2d error = txWorldTarget.value().minusExp(pose);
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

            if (t >= turn.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

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
        return localizer.getLastUpdateMethod() == Localizers.Methods.VISION;
    }

    /*private boolean isTwistUnderLimit(Twist2dDual<Time> twist) {
        double velX = twist.velocity().linearVel.x.value();
        double velY = twist.velocity().linearVel.y.value();
        double velAng = twist.velocity().angVel.value();
        return (velX * velX + velY * velY < Constants.LOCALIZATION_LINEAR_THRESHOLD_SQR) && (Math.abs(velAng) < Constants.LOCALIZATION_ANGULAR_THRESHOLD);
    }*/

    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        Pose2d absPose = localizer.getAbsolutePosition();

        if (absPose == null) {
            pose = pose.plus(twist.value());
        } else {
            pose = absPose;
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

    public class TelemetryLoggerCommand implements Command {
        private final Telemetry telemetry;

        public TelemetryLoggerCommand(Telemetry telemetry) {
            this.telemetry = telemetry;
        }

        @Override
        public Set<Subsystem> getRequirements() {
            return Collections.emptySet();
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
