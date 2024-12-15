/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetrySubsystem;

/*
 * This file contains a simple example "OpMode" for driving a robot.
 * An OpMode (Operation Mode) is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The OpMode to use can be selected on the FTC Driver Station.
 *
 * Hardware names are listed in the Constants file. You must correctly set all hardware names for each
 * motor/servo/etc. in the Driver Station for the code to find your devices.
 *
 * This code goes from the starting position (closest to net zone), turns right, moves, and drops
 * off a preloaded sample. That's it.
 *
 * Before starting this OpMode, the arm lift must be in its lowest position and the extension fully
 * retracted. The robot should also be facing forward, or else the field-centric driving will be off until reset.
 *
 * See MecanumTeleOpMode for more basic info.
 *
 *
 * The initialize() method runs when the "init" button is pressed on the Driver Station.
 * Everything afterward is handled by commands.
 *
 *
 * Tips:
 * If you don't understand what a particular method does, then hover your mouse over it to get some info and a short description.
 * If you click on a variable/method, then all uses of it in the file will be highlighted.
 * Click on a variable/method and press Ctrl+B (Cmd+B on mac) to be taken to where it was originally declared.
 * In addition, do that twice for a list of everywhere that variable/method is used.
 * You can also do that by holding ctrl while you click on the variable/method.
 */

@Autonomous(name="Low-Tech Auto-OpMode", group="Autonomous OpMode")
public class LowTechAutoOpMode extends CommandOpMode {

    // Hardware Variables
    private DriveSubsystem driveSubsystem;
    private ArmSubsystem armSubsystem;
    private TelemetrySubsystem telemetrySubsystem;


    // This is run when the "INIT" button is pressed on the Driver Station.
    @Override
    public void initialize() {
        // Initialize hardware
        driveSubsystem = new DriveSubsystem(hardwareMap, Constants.DRIVE_POWER_MULTIPLIER_MED);
        armSubsystem = new ArmSubsystem(hardwareMap);
        telemetrySubsystem = new TelemetrySubsystem(this, driveSubsystem, armSubsystem);

        // Mark subsystems to not zero again once the next opmode begins (teleop)
        ResetZeroState.markToNotZeroOnInit();


        // Schedule autonomous sequence
        SequentialCommandGroup routine = new SequentialCommandGroup(
                new WaitCommand(1000),
                //new InstantCommand(() -> armSubsystem.applyNamedPosition("intake stage 1")),
                //new WaitCommand(2000),
                //driveForTime(500, 0, 1, 0),
                driveForTime(500, -1, 0, 0),
                //new InstantCommand(armSubsystem::startOuttake),
                //new WaitCommand(2000),
                //new InstantCommand(() -> armSubsystem.applyNamedPosition("stow")),
                //new WaitCommand(2000),
                driveForTime(2000, 0, -1, 0),
                driveForTime(1000, 0, 1, 0),
                driveForTime(2000, 1, 0, 0),
                driveForTime(1500, 0, 1, 0),
                new InstantCommand(armSubsystem::moveMotorsToZero)
        );

        schedule(routine);


        // Default Commands

        telemetrySubsystem.setDefaultCommand(new RunCommand(telemetrySubsystem::reportTelemetry, telemetrySubsystem));
    }

    // Helper Methods

    private SequentialCommandGroup driveForTime(long millis, double axial, double lateral, double yaw) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.drive(axial, lateral, yaw)),
                new WaitCommand(millis),
                new InstantCommand(() -> driveSubsystem.drive(0, 0, 0)),
                new WaitCommand(500)
        );
    }
}
