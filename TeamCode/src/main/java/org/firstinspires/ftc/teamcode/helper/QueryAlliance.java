package org.firstinspires.ftc.teamcode.helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * A class that provides a convenient way to ask the drivers which alliance to run the robot for.
 * The methods and variables of this class are static so that the selected alliance persists between opmodes.
 */
public class QueryAlliance {
    private static boolean isBlueAlliance = false;

    /**
     * Query the drivers, asking which alliance the robot is on.
     * <p>Use {@link #isOnBlueAlliance()} to get the result. This method also returns the result of that method for convenience.</p>
     * <p>THIS SHOULD ALWAYS BE RUN LAST IN INITIALIZATION, as it waits for user input and could potentially hang until the end of init.</p>
     */
    public static boolean query(LinearOpMode opMode) {
        Gamepad gamepad = opMode.gamepad1;

        opMode.telemetry.addLine("Which alliance are you on?");
        opMode.telemetry.addData("Press south button (A/cross)", "Blue Alliance");
        opMode.telemetry.addData("Press east button (B/circle)", "Red Alliance");
        opMode.telemetry.addData("Current alliance", getCurrentAllianceName());
        opMode.telemetry.update();

        int button = getButton(gamepad);
        for (; button == -1 && opMode.opModeInInit(); button = getButton(gamepad)) {
            opMode.sleep(50);
        }

        if (button != -1) isBlueAlliance = button == 0;

        opMode.telemetry.addData("Your alliance", getCurrentAllianceName());
        opMode.telemetry.update();

        return isOnBlueAlliance();
    }

    private static int getButton(Gamepad gamepad) {
        return gamepad.a ? 0 : gamepad.b ? 1 : -1;
    }

    /**
     * @see #query(LinearOpMode)
     */
    public static boolean isOnBlueAlliance() {
        return isBlueAlliance;
    }

    public static String getCurrentAllianceName() {
        return isOnBlueAlliance() ? "Blue Alliance" : "Red Alliance";
    }
}
