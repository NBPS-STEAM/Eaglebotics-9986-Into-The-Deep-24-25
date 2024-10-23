package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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

    // Private instance variables (private variables that are in an instance of this class)
    private final Motor frontLeftMotor; // 'final' means that this variable will never be set again after it is set in the constructor.
    private final Motor frontRightMotor;
    private final Motor backLeftMotor;
    private final Motor backRightMotor;
    private final IMU imu;

    private double powerMultiplier;
    private boolean isFieldCentric = true;


    // Constructor methods
    public DriveSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, 1.0);
    }

    public DriveSubsystem(HardwareMap hardwareMap, double powerMultiplier) {
        this.powerMultiplier = powerMultiplier;

        this.frontLeftMotor = new Motor(hardwareMap, Constants.NAME_DRIVE_FL);
        this.frontRightMotor = new Motor(hardwareMap, Constants.NAME_DRIVE_FR);
        this.backLeftMotor = new Motor(hardwareMap, Constants.NAME_DRIVE_BL);
        this.backRightMotor = new Motor(hardwareMap, Constants.NAME_DRIVE_BR);

        this.frontLeftMotor.setInverted(false);
        this.backLeftMotor.setInverted(false);
        this.frontRightMotor.setInverted(true);
        this.backRightMotor.setInverted(true);

        this.frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.backLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.frontLeftMotor.setRunMode(Motor.RunMode.RawPower);
        this.backLeftMotor.setRunMode(Motor.RunMode.RawPower);
        this.frontRightMotor.setRunMode(Motor.RunMode.RawPower);
        this.backRightMotor.setRunMode(Motor.RunMode.RawPower);

        this.frontLeftMotor.resetEncoder();
        this.backLeftMotor.resetEncoder();
        this.frontRightMotor.resetEncoder();
        this.backRightMotor.resetEncoder();

        // Retrieve the IMU from the hardware map
        this.imu = hardwareMap.get(IMU.class, Constants.NAME_IMU);
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        this.imu.initialize(parameters);
    }


    // Methods

    public void drive(GamepadEx gamepad) {
        drive(gamepad.getLeftY(), gamepad.getLeftX(), gamepad.getRightX());
    }
    public void drive(Gamepad gamepad) {
        drive(-gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x);
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
        frontLeftMotor.set(power.getFrontLeftValue() * powerMultiplier);
        frontRightMotor.set(power.getFrontRightValue() * powerMultiplier);
        backLeftMotor.set(power.getBackLeftValue() * powerMultiplier);
        backRightMotor.set(power.getBackRightValue() * powerMultiplier);
    }

    public void setPower(double power) {
        frontLeftMotor.set(power * powerMultiplier);
        frontRightMotor.set(power * powerMultiplier);
        backLeftMotor.set(power * powerMultiplier);
        backRightMotor.set(power * powerMultiplier);
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
    public void zeroMotors() {
        frontLeftMotor.resetEncoder();
        frontRightMotor.resetEncoder();
        backLeftMotor.resetEncoder();
        backRightMotor.resetEncoder();
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
                frontLeftMotor.get(),
                frontRightMotor.get(),
                backLeftMotor.get(),
                backRightMotor.get());
    }

    // Getter methods

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
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
