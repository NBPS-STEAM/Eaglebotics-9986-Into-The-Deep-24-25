package org.firstinspires.ftc.teamcode.helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * A class that provides a convenient way to prompt drivers, such as asking which alliance to run the robot for.
 * <p>The methods and variables of this class are static, so they can be used from anywhere.</p>
 */
public class DriverPrompter {

    /**
     * Query the drivers, asking some yes or no question.
     * <p>Returns the value of {@code defaultResult} if no response is given.</p>
     * <p>THIS SHOULD ALWAYS BE RUN LAST IN INITIALIZATION, as it waits for user input and could potentially hang until the end of init.</p>
     */
    public static boolean queryBoolean(LinearOpMode opMode, boolean defaultResult, String prompt, String shortName) {
        Gamepad gamepad = opMode.gamepad1;

        opMode.telemetry.addLine();
        opMode.telemetry.addLine(prompt);
        opMode.telemetry.addData("Press south button (A/cross)", "Yes");
        opMode.telemetry.addData("Press east button (B/circle)", "No");
        opMode.telemetry.addData("Default", defaultResult);
        opMode.telemetry.update();

        Boolean button = getButton(gamepad);
        while (button == null && opMode.opModeInInit()) {
            opMode.sleep(50);
            button = getButton(gamepad);
        }

        boolean result = defaultResult;
        if (button != null) result = button;

        opMode.telemetry.addData(shortName, result).setRetained(true);
        opMode.telemetry.update();

        return result;
    }

    private static boolean isBlueAlliance = false;

    /**
     * Query the drivers, asking which alliance the robot is on.
     * <p>Use {@link #isOnBlueAlliance()} to get the result. This method also returns the result of that method for convenience.</p>
     * <p>THIS SHOULD ALWAYS BE RUN LAST IN INITIALIZATION, as it waits for user input and could potentially hang until the end of init.</p>
     * <p>The result of this is stored in a static variable so that the selected alliance persists between opmodes.</p>
     */
    public static boolean queryAlliance(LinearOpMode opMode) {
        Gamepad gamepad = opMode.gamepad1;

        opMode.telemetry.addLine();
        opMode.telemetry.addLine("Which alliance are you on?");
        opMode.telemetry.addData("Press south button (A/cross)", "Blue Alliance");
        opMode.telemetry.addData("Press east button (B/circle)", "Red Alliance");
        opMode.telemetry.addData("Current alliance", getCurrentAllianceName());
        opMode.telemetry.update();

        Boolean button = getButton(gamepad);
        while (button == null && opMode.opModeInInit()) {
            opMode.sleep(50);
            button = getButton(gamepad);
        }

        if (button != null) isBlueAlliance = button;

        opMode.telemetry.addData("Your alliance", getCurrentAllianceName()).setRetained(true);
        opMode.telemetry.update();

        return isOnBlueAlliance();
    }

    private static Boolean getButton(Gamepad gamepad) {
        if (gamepad.a) return true;
        if (gamepad.b) return false;
        return null;
    }

    /**
     * @see #queryAlliance(LinearOpMode)
     */
    public static boolean isOnBlueAlliance() {
        return isBlueAlliance;
    }

    public static String getCurrentAllianceName() {
        return isOnBlueAlliance() ? "Blue Alliance" : "Red Alliance";
    }
}
