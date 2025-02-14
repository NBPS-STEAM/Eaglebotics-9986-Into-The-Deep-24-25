package org.example;

import com.acmerobotics.roadrunner.*;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.entity.Entity;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.lang.Math;
import java.util.Arrays;

/**
 * MeepMeep is the name of the system that visualizes Road Runner paths.
 * To use it, first program your routine into this class, then change your run configuration (top right)
 * from TeamCode to run-rr-visualizer and click the green arrow next to it.
 * If you don't have the run-rr-visualizer option, go to the link below for instructions on adding it.
 * For more information, see https://github.com/acmerobotics/MeepMeep
 */
public class MeepMeepTesting {
    private static Vector2d legacyTransform(Vector2d vec) {
        return new Vector2d(-(vec.y - 8), vec.x - 61);
    }
    private static Pose2d legacyTransform(Pose2d pose) {
        return new Pose2d(legacyTransform(pose.position), pose.heading.plus(Math.PI / 2));
    }

    private static Vector2d legacyTransform2(Vector2d vec) {
        return new Vector2d(-(vec.y + 12), vec.x - 61);
    }
    private static Pose2d legacyTransform2(Pose2d pose) {
        return new Pose2d(legacyTransform2(pose.position), pose.heading.plus(Math.PI / 2));
    }
    private static double adaptAngle(double angle) {
        return angle + Math.PI / 2;
    }

    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(simBasketsV2(meepMeep))
                //.addEntity(simSpecimensV2(meepMeep))
                .start();
    }

    private static Entity simSpecimensV2(MeepMeep meepMeep) {
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.PI * 1.2, Math.PI * 1.2, 12)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(legacyTransform2(new Pose2d(0, -24, 0)))
                .waitSeconds(2)
                // Score preload
                //.afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                //.afterTime(0.0, () -> armSubsystem.applyNamedPosition("specimen high"))
                .strafeTo(legacyTransform2(new Vector2d(24, -20)))
                //.afterTime(0.0, () -> armSubsystem.applyNamedPosition("specimen low"))
                //.stopAndAdd(armSubsystem.yieldForRotationTarget())
                //.afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .waitSeconds(0.3)

                // Move left sample into observation zone
                .setTangent(adaptAngle(-Math.PI * 3 / 4))
                .splineToSplineHeading(legacyTransform2(new Pose2d(28, -52, -Math.PI / 4)), adaptAngle(-Math.PI / 4))
                //.stopAndAdd(() -> armSubsystem.applyNamedPosition("intake ground"))
                //.stopAndAdd(() -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                //.stopAndAdd(armSubsystem.yieldForRotationTarget())
                //.stopAndAdd(() -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                .waitSeconds(0.3)
                //.stopAndAdd(() -> armSubsystem.applyNamedPosition("stow"))
                //.afterTime(1.0, () -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .turn(Math.PI * 4)

                // Park in observation zone
                .strafeTo(legacyTransform2(new Vector2d(6, -24)))
                //.afterTime(0, () -> armSubsystem.applyNamedPosition("stow"))
                .strafeTo(legacyTransform2(new Vector2d(6, -60)))
                .build());

        return myBot;
    }

    private static Entity simBasketsV2(MeepMeep meepMeep) {
        final Vector2d BASKET_POS = legacyTransform(new Vector2d(7.75, 63));
        final Pose2d BASKET_POSE = legacyTransform(new Pose2d(7.75, 63, Math.PI * 3 / 4));

        final Pose2d KEYPOINT_1 = legacyTransform(new Pose2d(21, 62, 0));
        final Pose2d KEYPOINT_2 = legacyTransform(new Pose2d(21.5, 71, 0));
        final Pose2d KEYPOINT_3 = legacyTransform(new Pose2d(40, 54, Math.PI / 2));
        final Pose2d KEYPOINT_3B = legacyTransform(new Pose2d(40, 58, Math.PI / 2));
        final Pose2d KEYPOINT_4 = legacyTransform(new Pose2d(52, 48, 0));
        final Pose2d KEYPOINT_5 = legacyTransform(new Pose2d(52, 30, -Math.PI / 2));

        final double INTAKE_DELAY = 0.3;
        final double OUTTAKE_DELAY = 0.3;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.PI * 1.2, Math.PI * 1.2, 12)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(legacyTransform(new Pose2d(0, 36, 0)))
                // Score preload
                //.afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                //.afterTime(0.0, () -> armSubsystem.applyNamedPosition("basket high"))
                .strafeTo(BASKET_POS)
                //.stopAndAdd(armSubsystem.yieldForRaiseTarget())
                //.stopAndAdd(armSubsystem.yieldForRotationTarget())
                .turnTo(BASKET_POSE.heading)
                //.afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .waitSeconds(OUTTAKE_DELAY)

                // Score right spike mark sample
                //.afterTime(0.5, () -> armSubsystem.applyNamedPosition("intake ground-high"))
                .strafeToLinearHeading(KEYPOINT_1.position, KEYPOINT_1.heading)
                //.stopAndAdd(armSubsystem.yieldForRotationTarget(55))
                //.stopAndAdd(() -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                .waitSeconds(INTAKE_DELAY)
                //.stopAndAdd(() -> armSubsystem.applyNamedPosition("basket high"))
                //.stopAndAdd(armSubsystem.yieldForRotationTarget())
                .strafeToLinearHeading(BASKET_POSE.position, BASKET_POSE.heading)
                //.stopAndAdd(() -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .waitSeconds(OUTTAKE_DELAY)

                // Score center spike mark sample
                //.afterTime(0.5, () -> armSubsystem.applyNamedPosition("intake ground-high"))
                .strafeToLinearHeading(KEYPOINT_2.position, KEYPOINT_2.heading)
                //.stopAndAdd(armSubsystem.yieldForRotationTarget(55))
                //.stopAndAdd(() -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                .waitSeconds(INTAKE_DELAY)
                //.stopAndAdd(() -> armSubsystem.applyNamedPosition("basket high"))
                //.stopAndAdd(armSubsystem.yieldForRotationTarget())
                .strafeToLinearHeading(BASKET_POSE.position, BASKET_POSE.heading)
                //.stopAndAdd(() -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .waitSeconds(OUTTAKE_DELAY)

                // Score left spike mark sample
                //.afterTime(0.5, () -> armSubsystem.applyNamedPosition("intake ground-high"))
                .strafeToLinearHeading(KEYPOINT_3.position, KEYPOINT_3.heading)
                //.stopAndAdd(armSubsystem.yieldForRotationTarget(55))
                .strafeToLinearHeading(KEYPOINT_3B.position, KEYPOINT_3B.heading)
                .waitSeconds(0.1)
                //.stopAndAdd(() -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                .waitSeconds(INTAKE_DELAY)
                .strafeToLinearHeading(KEYPOINT_3.position, KEYPOINT_3.heading)
                //.stopAndAdd(blockIfTimeout(3.0)) // If there aren't at least 3 seconds remaining at this point, the routine pauses indefinitely.
                //.stopAndAdd(() -> armSubsystem.applyNamedPosition("basket high"))
                //.stopAndAdd(armSubsystem.yieldForRotationTarget())
                .strafeToLinearHeading(BASKET_POSE.position, BASKET_POSE.heading)
                //.stopAndAdd(() -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .waitSeconds(OUTTAKE_DELAY)

                // Park in ascent zone and achieve Level 1 Ascent
                //.stopAndAdd(blockIfTimeout(2.0))
                //.afterTime(0.5, () -> armSubsystem.applyNamedPosition ("ascent level 1"))
                .strafeToLinearHeading(KEYPOINT_4.position, KEYPOINT_4.heading)
                .strafeToLinearHeading(KEYPOINT_5.position, KEYPOINT_5.heading)
                //.afterTime(0.0, () -> armSubsystem.setRotationPower(0.0))
                .build());

        return myBot;
    }
}
