package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import org.firstinspires.ftc.teamcode.helper.localization.Localizers;

public interface Localizer {
    /**
     * Update localizer and produce a Twist2dDual describing change in position (relative to last call) and absolute velocity.
     * <p>After running this, use {@link #getAbsolutePosition()} to get the absolute position (if available).</p>
     */
    Twist2dDual<Time> update();

    /**
     * Gets the absolute position determined by the localizer in the last call to {@link #update()}, if one is available.
     */
    default Pose2d getAbsolutePosition() {
        return null;
    }

    default Localizers.Methods getLastUpdateMethod() {
        return Localizers.Methods.UNSPECIFIED;
    }

    default void setAlliance(boolean isBlueAlliance) {}
}
