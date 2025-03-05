package org.firstinspires.ftc.teamcode.helper;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

public class SequentialCommandGroupFix extends SequentialCommandGroup {
    public SequentialCommandGroupFix(Command... commands) {
        super(commands);
    }

    @Override
    public void end(boolean interrupted) {
        try {
            addCommands(new InstantCommand());
        } catch (IllegalStateException e) {
            super.end(interrupted);
        }
    }
}
