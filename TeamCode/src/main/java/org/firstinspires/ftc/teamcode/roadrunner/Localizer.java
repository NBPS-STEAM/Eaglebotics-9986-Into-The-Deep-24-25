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
     * Gets the absolute position determined by the localizer in the last call to {@link #update()} or
     * {@link #calculateAbsolutePosition()}, if one is available.
     */
    default Pose2d getAbsolutePosition() {
        return null;
    }

    /**
     * Attempt to determine absolute position on the spot, if possible.
     * <p>Unless you have a specific reason to use this, you should usually use {@link #getAbsolutePosition()} instead.</p>
     */
    default Pose2d calculateAbsolutePosition() {
        return null;
    }

    /**
     * The technique used to localize in the last call to {@link #update()} or {@link #calculateAbsolutePosition()}
     */
    default Localizers.Methods getLastUpdateMethod() {
        return Localizers.Methods.UNSPECIFIED;
    }

    default Localizers.Status getLastUpdateStatus() {
        return Localizers.Status.OK;
    }

    default void setAlliance(boolean isBlueAlliance) {}
}
