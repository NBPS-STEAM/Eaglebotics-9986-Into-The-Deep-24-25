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

package org.firstinspires.ftc.teamcode.helper;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/*
 * Zero all motors and reset the zeroOnInit flags to true.
 */

@Autonomous(name="Reset Zero State", group="Setup OpMode")
public class ResetZeroState extends LinearOpMode {

    // Static variables
    // Static variables aren't reset between opmodes, only when the robot turns off.
    // This variable is used by the autonomous routine to prevent the robot from resetting after auto.
    private static boolean zeroDriveOnInit = true;
    private static boolean zeroHeadingOnInit = true;
    private static boolean zeroArmOnInit = true;

    private static Vector2d prevDrivePos = null;
    private static Double prevHeading = null;


    // This is run when the "INIT" button is pressed on the Driver Station.
    @Override
    public void runOpMode() throws InterruptedException {
        zeroAllSubsystems(hardwareMap);
    }

    public static void zeroAllSubsystems(HardwareMap hardwareMap) {
        resetZeroState();
        new DriveSubsystem(hardwareMap, Constants.DRIVE_POWER_MULTIPLIER_MED);
        new ArmSubsystem(hardwareMap);
    }


    public static void markToNotZeroOnInit() {
        markToNotZeroOnInit(null, null);
    }
    public static void markToNotZeroOnInit(Vector2d posToPreserve) {
        markToNotZeroOnInit(posToPreserve, null);
    }
    public static void markToNotZeroOnInit(Double headingToPreserve) {
        markToNotZeroOnInit(null, headingToPreserve);
    }
    public static void markToNotZeroOnInit(Pose2d poseToPreserve) {
        markToNotZeroOnInit(poseToPreserve.position, poseToPreserve.heading.toDouble());
    }
    public static void markToNotZeroOnInit(Vector2d posToPreserve, Double headingToPreserve) {
        setZeroDriveOnInit(false, posToPreserve);
        setZeroHeadingOnInit(false, headingToPreserve);
        setZeroArmOnInit(false);
    }

    public static void resetZeroState() {
        setZeroDriveOnInit(true);
        setZeroHeadingOnInit(true);
        setZeroArmOnInit(true);
    }

    public static void setZeroDriveOnInit(boolean doZero) {
        setZeroDriveOnInit(doZero, null);
    }
    public static void setZeroDriveOnInit(boolean doZero, Vector2d posToPreserve) {
        zeroDriveOnInit = doZero;
        prevDrivePos = doZero ? null : posToPreserve;
    }

    public static void setZeroHeadingOnInit(boolean doZero) {
        setZeroHeadingOnInit(doZero, null);
    }
    public static void setZeroHeadingOnInit(boolean doZero, Double headingToPreserve) {
        zeroHeadingOnInit = doZero;
        prevHeading = doZero ? null : headingToPreserve;
    }

    public static void setZeroArmOnInit(boolean doZero) {
        zeroArmOnInit = doZero;
    }

    /**
     * Returns the zeroDriveOnInit flag and resets it to true.
     */
    public static boolean shouldZeroDrive() {
        if (zeroDriveOnInit) {
            return true;
        }
        setZeroDriveOnInit(true);
        return false;
    }

    /**
     * Get the drive position that should be manually preserved if not zeroing (if provided, may be null).
     */
    public static Vector2d getPrevDrivePos() {
        return prevDrivePos;
    }

    /**
     * Returns the zeroHeadingOnInit flag and resets it to true.
     */
    public static boolean shouldZeroHeading() {
        if (zeroHeadingOnInit) {
            return true;
        }
        setZeroHeadingOnInit(true);
        return false;
    }

    /**
     * Get the heading that should be manually preserved if not zeroing (if provided, may be null).
     */
    public static Double getPrevHeading() {
        return prevHeading;
    }

    /**
     * Returns the zeroArmOnInit flag and resets it to true.
     */
    public static boolean shouldZeroArm() {
        if (zeroArmOnInit) {
            return true;
        }
        setZeroArmOnInit(true);
        return false;
    }
}
