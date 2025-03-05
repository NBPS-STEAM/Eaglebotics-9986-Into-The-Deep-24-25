package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.*;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.helper.RoadRunnerCommand;
import org.firstinspires.ftc.teamcode.helper.SequentialCommandGroupFix;
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
        CommandBase command = new SequentialCommandGroupFix(
                new RoadRunnerCommand(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(Constants.POS_SPECIMEN_APPROACH.position, Constants.POS_SPECIMEN_APPROACH.heading)
                        .build()),
                new InstantCommand(() -> armSubsystem.applyNamedPosition("intake ground")),
                new ParallelRaceGroup(
                        new WaitUntilCommand(armSubsystem::hasSampleInIntake),
                        new RoadRunnerCommand(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(Constants.POS_SPECIMEN_INTAKE.position, Constants.POS_SPECIMEN_INTAKE.heading)
                                .build())
                )
        );
        command.addRequirements(drive);
        return command;
    }


    public Command getScoreSpecimenCommand() {
        CommandBase command = new SequentialCommandGroupFix(
                //TODO
        );
        command.addRequirements(drive);
        return command;
    }


    /**
     * This command executes an on-the-fly path to autonomously drive and score in the high basket before
     * returning to the submersible zone.
     * <p>This command requires the drive subsystem.</p>
     * <p>Uses: driveSubsystem, armSubsystem</p>
     */
    public Command getScoreSampleCommand() {
        CommandBase command = new SequentialCommandGroupFix(
                new InstantCommand(() -> armSubsystem.lockSetPosition(false)),
                new InstantCommand(() -> armSubsystem.applyNamedPosition("basket high", true, true)),
                getDriveToBasketPath(),
                new InstantCommand(armSubsystem::startOuttake),
                new WaitCommand(400),
                new InstantCommand(() -> armSubsystem.lockSetPosition(false)),
                getDriveFromBasketPath()
        );
        command.addRequirements(drive);
        return command;
    }

    private Command getDriveToBasketPath() {
        return new RoadRunnerCommand(() -> drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(Constants.POS_BASKETS_SCORE.position, Constants.POS_BASKETS_SCORE.heading)
                .build());//.withVisionCheck(drive, Constants.POS_BASKETS_SCORE);
    }

    private Command getDriveFromBasketPath() {
        return new RoadRunnerCommand(() -> drive.actionBuilder(drive.pose)
                .strafeTo(Constants.POS_INTAKE_APPROACH.position)
                .build());
    }
}
