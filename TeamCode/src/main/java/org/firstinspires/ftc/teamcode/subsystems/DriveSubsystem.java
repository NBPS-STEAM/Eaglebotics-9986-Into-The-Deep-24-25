package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Calculations;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.helper.QuadMotorValues;

/**
 * This is a class for a "subsystem" of driving motors. It records four
 * motors and can apply a set of power values to all four motors at once.
 * It also has some other features to control them like a drivetrain.
 */
public class DriveSubsystem extends SubsystemBase {

    // Static variables
    // Static variables aren't reset between opmodes, only when the robot turns off.
    // This variable is used by the autonomous routine to prevent the robot from resetting after auto.
    public static boolean zeroDriveOnInit = true;
    public static boolean zeroHeadingOnInit = true;

    // Private instance variables (private variables that are in an instance of this class)
    private final DcMotor frontLeftMotor; // 'final' means that this variable will never be set again after it is set in the constructor.
    private final DcMotor frontRightMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor backRightMotor;
    private final IMU imu;

    private double powerMultiplier;
    private boolean isFieldCentric = true;


    // Constructor methods
    public DriveSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, 1.0);
    }

    public DriveSubsystem(HardwareMap hardwareMap, double powerMultiplier) {
        this.powerMultiplier = powerMultiplier;

        this.frontLeftMotor = hardwareMap.get(DcMotor.class, Constants.NAME_DRIVE_FL);
        this.frontRightMotor = hardwareMap.get(DcMotor.class, Constants.NAME_DRIVE_FR);
        this.backLeftMotor = hardwareMap.get(DcMotor.class, Constants.NAME_DRIVE_BL);
        this.backRightMotor = hardwareMap.get(DcMotor.class, Constants.NAME_DRIVE_BR);

        this.frontLeftMotor.setDirection(Constants.DIRECTION_DRIVE_FL);
        this.backLeftMotor.setDirection(Constants.DIRECTION_DRIVE_BL);
        this.frontRightMotor.setDirection(Constants.DIRECTION_DRIVE_FR);
        this.backRightMotor.setDirection(Constants.DIRECTION_DRIVE_BR);

        this.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Retrieve the IMU from the hardware map
        this.imu = hardwareMap.get(IMU.class, Constants.NAME_IMU);
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                Constants.IMU_HUB_LOGO_DIRECTION,
                Constants.IMU_HUB_USB_DIRECTION));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        this.imu.initialize(parameters);

        // Zero unless told not to
        if (zeroDriveOnInit) {
            this.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            zeroHeading();
        }
        zeroDriveOnInit = true;

        if (zeroHeadingOnInit) {
            zeroHeading();
        }
        zeroHeadingOnInit = true;

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    // Methods

    public void drive(GamepadEx gamepad) {
        drive(gamepad.getLeftY(), gamepad.getLeftX(), gamepad.getRightX());
    }
    public void drive(Gamepad gamepad) {
        drive(-gamepad.left_stick_y, -gamepad.left_stick_x, gamepad.right_stick_x);
    }
    public void drive(double axial, double lateral, double yaw) {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        QuadMotorValues<Double> drivePower;
        if (isFieldCentric) {
            drivePower = Calculations.mecanumDriveFieldCentric(axial, lateral, yaw, heading);
        } else {
            drivePower = Calculations.mecanumDriveRobotCentric(axial, lateral, yaw);
        }
        setPower(drivePower);
    }

    public void setFullSpeed() {
        powerMultiplier = Constants.DRIVE_POWER_MULTIPLIER;
    }

    public void setMediumSpeed() {
        powerMultiplier = Constants.DRIVE_POWER_MULTIPLIER_MED;
    }

    public void setSlowSpeed() {
        powerMultiplier = Constants.DRIVE_POWER_MULTIPLIER_SLOW;
    }

    public void toggleIsFieldCentric() {
        isFieldCentric = !isFieldCentric;
    }

    public void setPower(QuadMotorValues<Double> power) {
        frontLeftMotor.setPower(power.getFrontLeftValue() * powerMultiplier);
        frontRightMotor.setPower(power.getFrontRightValue() * powerMultiplier);
        backLeftMotor.setPower(power.getBackLeftValue() * powerMultiplier);
        backRightMotor.setPower(power.getBackRightValue() * powerMultiplier);
    }

    public void setPower(double power) {
        frontLeftMotor.setPower(power * powerMultiplier);
        frontRightMotor.setPower(power * powerMultiplier);
        backLeftMotor.setPower(power * powerMultiplier);
        backRightMotor.setPower(power * powerMultiplier);
    }

    public void setTargetPosition(QuadMotorValues<Integer> position) {
        frontLeftMotor.setTargetPosition(position.getFrontLeftValue());
        frontRightMotor.setTargetPosition(position.getFrontRightValue());
        backLeftMotor.setTargetPosition(position.getBackLeftValue());
        backRightMotor.setTargetPosition(position.getBackRightValue());
    }

    public void setTargetPosition(int position) {
        frontLeftMotor.setTargetPosition(position);
        frontRightMotor.setTargetPosition(position);
        backLeftMotor.setTargetPosition(position);
        backRightMotor.setTargetPosition(position);
    }

    // Method to zero all four motors
    public void zeroMotors(DcMotor.RunMode nextMode) {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(nextMode);
        backLeftMotor.setMode(nextMode);
        frontRightMotor.setMode(nextMode);
        backRightMotor.setMode(nextMode);
    }

    // Method to zero IMU heading
    public void zeroHeading() {
        imu.resetYaw();
    }


    // Get motor properties

    public QuadMotorValues<Double> getMotorPositions() {
        return new QuadMotorValues<>(
                Calculations.encoderToInchesDrive(frontLeftMotor.getCurrentPosition()),
                Calculations.encoderToInchesDrive(frontRightMotor.getCurrentPosition()),
                Calculations.encoderToInchesDrive(backLeftMotor.getCurrentPosition()),
                Calculations.encoderToInchesDrive(backRightMotor.getCurrentPosition()));
    }

    public QuadMotorValues<Integer> getMotorPositionsUnscaled() {
        return new QuadMotorValues<>(
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(),
                backLeftMotor.getCurrentPosition(),
                backRightMotor.getCurrentPosition());
    }

    public QuadMotorValues<Double> getMotorPowers() {
        return new QuadMotorValues<>(
                frontLeftMotor.getPower(),
                frontRightMotor.getPower(),
                backLeftMotor.getPower(),
                backRightMotor.getPower());
    }

    // Getter methods

    public double getHeading() {
        return getHeading(AngleUnit.RADIANS);
    }

    public double getHeading(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    public double getPowerMultiplier() {
        return powerMultiplier;
    }

    public boolean getIsFieldCentric() {
        return isFieldCentric;
    }


    // Setter methods

    public void setPowerMultiplier(double multiplier) {
        powerMultiplier = multiplier;
    }

    public void setIsFieldCentric(boolean isFieldCentric) {
        this.isFieldCentric = isFieldCentric;
    }
}
