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
     * <p>Blocks until either a response is received or init ends. THIS SHOULD ALWAYS BE RUN LAST IN INITIALIZATION,
     * as it waits for user input and could potentially hang until the end of init.</p>
     */
    public static boolean queryBoolean(LinearOpMode opMode, boolean defaultResult, String prompt, String shortName) {
        Gamepad gamepad = opMode.gamepad1;

        opMode.telemetry.addLine();
        opMode.telemetry.addLine(prompt);
        opMode.telemetry.addData("Press south button (A/cross)", "Yes");
        opMode.telemetry.addData("Press east button (B/circle)", "No");
        opMode.telemetry.addData("Default", defaultResult);
        opMode.telemetry.update();

        // Wait until valid button is pressed
        Boolean button = getButton(gamepad);
        while (button == null && opMode.opModeInInit()) {
            opMode.sleep(50);
            button = getButton(gamepad);
        }
        // Wait for button to be released
        while (getButton(gamepad) != null && opMode.opModeInInit()) {
            opMode.sleep(50);
        }

        boolean result = defaultResult;
        if (button != null) result = button;

        opMode.telemetry.addData(shortName, result).setRetained(true);
        opMode.telemetry.update();

        return result;
    }

    private static boolean isBlueAlliance = false;
    private static boolean allianceFromDriver = false;

    /**
     * Query the drivers, asking which alliance the robot is on.
     * <p>Use {@link #isOnBlueAlliance()} to get the result. This method also returns the result of that method for convenience.</p>
     * <p>Blocks until either a response is received or init ends. THIS SHOULD ALWAYS BE RUN LAST IN INITIALIZATION,
     * as it waits for user input and could potentially hang until the end of init.</p>
     * <p>The result of this is stored in a static variable so that the selected alliance persists between opmodes.</p>
     * <p>The alliance name displayed on the Driver Hub will automatically update with the stored alliance color.</p>
     * @see #isOnBlueAlliance()
     * @see #setOnBlueAlliance(boolean)
     */
    public static boolean queryAlliance(LinearOpMode opMode) {
        Gamepad gamepad = opMode.gamepad1;

        opMode.telemetry.addLine();
        opMode.telemetry.addLine("Which alliance are you on?");
        opMode.telemetry.addData("Press south button (A/cross)", "Blue Alliance");
        opMode.telemetry.addData("Press east button (B/circle)", "Red Alliance");
        opMode.telemetry.addData("Current alliance", getCurrentAllianceName());
        opMode.telemetry.update();

        // Wait until valid button is pressed
        Boolean button = getButton(gamepad);
        while (button == null && opMode.opModeInInit()) {
            opMode.sleep(50);
            button = getButton(gamepad);
        }
        // Wait for button to be released
        while (getButton(gamepad) != null && opMode.opModeInInit()) {
            opMode.sleep(50);
        }

        allianceFromDriver = button != null;
        if (allianceFromDriver) isBlueAlliance = button;

        opMode.telemetry.addData("Your alliance", DriverPrompter::getCurrentAllianceName).setRetained(true);
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

    /**
     * Manually set the stored alliance color.
     */
    public static void setOnBlueAlliance(boolean isBlueAlliance) {
        DriverPrompter.isBlueAlliance = isBlueAlliance;
    }

    /**
     * Returns true if the last call to {@link #queryAlliance(LinearOpMode)} resulted in the driver making a selection.
     * <p>If false, then that means the call ended early and the alliance was decided by the default.</p>
     */
    public static boolean wasAllianceFromDriver() {
        return allianceFromDriver;
    }

    public static String getCurrentAllianceName() {
        return isOnBlueAlliance() ? "Blue Alliance" : "Red Alliance";
    }
}
