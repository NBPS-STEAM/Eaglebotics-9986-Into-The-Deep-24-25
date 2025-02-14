package org.firstinspires.ftc.teamcode.helper.localization;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.*;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.roadrunner.messages.MecanumLocalizerInputsMessage;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystemRRVision;

public class DriveLocalizer implements Localizer {

    public final DriveSubsystemRRVision drive;
    public final Encoder leftFront, leftBack, rightBack, rightFront;
    public final IMU imu;

    private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
    private Rotation2d lastHeading;
    private boolean initialized;

    public DriveLocalizer(DriveSubsystemRRVision drive) {
        this.drive = drive;

        this.leftFront = new OverflowEncoder(new RawEncoder(this.drive.leftFront));
        this.leftBack = new OverflowEncoder(new RawEncoder(this.drive.leftBack));
        this.rightBack = new OverflowEncoder(new RawEncoder(this.drive.rightBack));
        this.rightFront = new OverflowEncoder(new RawEncoder(this.drive.rightFront));

        imu = this.drive.lazyImu.get();

        this.leftFront.setDirection(Constants.DIRECTION_DRIVE_FL);
        this.leftBack.setDirection(Constants.DIRECTION_DRIVE_BL);
        this.rightBack.setDirection(Constants.DIRECTION_DRIVE_BR);
        this.rightFront.setDirection(Constants.DIRECTION_DRIVE_FR);
    }

    @Override
    public Twist2dDual<Time> update() {
        PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
        PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
        PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
        PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

        //FlightRecorder.write("MECANUM_LOCALIZER_INPUTS", new MecanumLocalizerInputsMessage(
        //        leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles));

        Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

        if (!initialized) {
            initialized = true;

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            lastHeading = heading;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        double headingDelta = heading.minus(lastHeading);
        Twist2dDual<Time> twist = drive.kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                new DualNum<Time>(new double[]{
                        (leftFrontPosVel.position - lastLeftFrontPos),
                        leftFrontPosVel.velocity,
                }).times(drive.PARAMS.inPerTick),
                new DualNum<Time>(new double[]{
                        (leftBackPosVel.position - lastLeftBackPos),
                        leftBackPosVel.velocity,
                }).times(drive.PARAMS.inPerTick),
                new DualNum<Time>(new double[]{
                        (rightBackPosVel.position - lastRightBackPos),
                        rightBackPosVel.velocity,
                }).times(drive.PARAMS.inPerTick),
                new DualNum<Time>(new double[]{
                        (rightFrontPosVel.position - lastRightFrontPos),
                        rightFrontPosVel.velocity,
                }).times(drive.PARAMS.inPerTick)
        ));

        lastLeftFrontPos = leftFrontPosVel.position;
        lastLeftBackPos = leftBackPosVel.position;
        lastRightBackPos = rightBackPosVel.position;
        lastRightFrontPos = rightFrontPosVel.position;

        lastHeading = heading;

        return new Twist2dDual<>(
                twist.line,
                DualNum.cons(headingDelta, twist.angle.drop(1))
        );
    }

    @Override
    public Localizers.Methods getLastUpdateMethod() {
        return Localizers.Methods.ENCODERS;
    }
}
