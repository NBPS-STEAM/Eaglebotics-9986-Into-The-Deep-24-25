package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.helper.RoadRunnerCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystemRRVision;

/**
 * HIGHLY RECOMMENDED that if autonomous driving routines are used in teleop, such is done immediately after localizing with vision.
 * Otherwise, the localization will almost certainly be off if it's relying only on encoders.
 */
public final class AutoCommands {
    private final DriveSubsystemRRVision drive;
    private final ArmSubsystem armSubsystem;

    public AutoCommands(DriveSubsystemRRVision driveSubsystem, ArmSubsystem armSubsystem) {
        this.drive = driveSubsystem;
        this.armSubsystem = armSubsystem;
    }

    public Command getIntakeSpecimenCommand() {
        CommandBase command = new SequentialCommandGroup(
                getDriveToBasketPath1(),
                getDriveToBasketPath2()
        );
        command.addRequirements(drive);
        return command;
    }

    public Command getScoreSpecimenCommand() {
        return new InstantCommand(); //TODO
    }

    /**
     * This command executes an on-the-fly path to autonomously drive and score in the high basket before
     * returning to the submersible zone.
     * <p>This command requires the drive subsystem.</p>
     * <p>Uses: driveSubsystem, armSubsystem</p>
     */
    public Command getScoreSampleCommand() {
        CommandBase command = new SequentialCommandGroup(
                getDriveToBasketPath1(),
                getDriveToBasketPath2()
        );
        command.addRequirements(drive);
        return command;
    }

    private Command getDriveToBasketPath1() {
        // Second path (drive to basket)
        return new RoadRunnerCommand(() -> drive.actionBuilder(drive.pose)
                .stopAndAdd(() -> armSubsystem.lockSetPosition(false))
                .stopAndAdd(() -> armSubsystem.applyNamedPosition("basket high", true, true))
                .strafeToLinearHeading(Constants.POS_BASKETS_SCORE.position, Constants.POS_BASKETS_SCORE.heading)
                .build()).withVisionCheck(drive, Constants.POS_BASKETS_SCORE);
    }

    private Command getDriveToBasketPath2() {
        // Third path (score and return)
        return new RoadRunnerCommand(() -> drive.actionBuilder(drive.pose)
                .stopAndAdd(armSubsystem::startOuttake)
                .waitSeconds(0.4)
                .afterTime(0.0, () -> armSubsystem.lockSetPosition(false))
                .afterTime(0.5, () -> armSubsystem.applyNamedPosition("compact"))
                .strafeTo(Constants.POS_INTAKE_APPROACH.position)
                .build());
    }
}
