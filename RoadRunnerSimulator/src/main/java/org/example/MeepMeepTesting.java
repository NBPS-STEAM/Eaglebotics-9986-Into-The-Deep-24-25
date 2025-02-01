package org.example;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.entity.Entity;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

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
    private static double legacyRotate(double angle) {
        return angle + Math.PI / 2;
    }

    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(simBasketsV2(meepMeep))
                .addEntity(simSpecimensV2(meepMeep))
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
                .setTangent(legacyRotate(-Math.PI * 3 / 4))
                .splineToSplineHeading(legacyTransform2(new Pose2d(28, -52, -Math.PI / 4)), legacyRotate(-Math.PI / 4))
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

        final Pose2d KEYPOINT_1 = legacyTransform(new Pose2d(18, 54, 0));
        final double KEYPOINT_1_TAN = Math.atan2(KEYPOINT_1.position.x - BASKET_POS.x, KEYPOINT_1.position.y - BASKET_POS.y);
        final Pose2d KEYPOINT_2 = legacyTransform(new Pose2d(18, 66, 0));
        final Pose2d KEYPOINT_3 = legacyTransform(new Pose2d(36, 60, Math.PI / 2));
        final Pose2d KEYPOINT_4 = legacyTransform(new Pose2d(42, 63, -Math.PI / 2));
        final Pose2d KEYPOINT_5 = legacyTransform(new Pose2d(52, 36, -Math.PI / 2));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.PI * 0.8, Math.PI * 0.9, 12)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(legacyTransform(new Pose2d(0, 36, 0)))
                .waitSeconds(2)
                // Score preload
                //.afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                //.afterTime(0.0, () -> armSubsystem.applyNamedPosition("basket high"))
                .strafeTo(BASKET_POS)
                //.stopAndAdd(armSubsystem.yieldForRaiseTarget())
                .turnTo(BASKET_POSE.heading)
                //.afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .waitSeconds(0.4)

                // Score right spike mark sample
                //.afterTime(0.5, () -> armSubsystem.applyNamedPosition("intake ground-high"))
                .setTangent(KEYPOINT_1_TAN)
                .splineToLinearHeading(KEYPOINT_1, KEYPOINT_1_TAN)
                //.stopAndAdd(armSubsystem.yieldForRotationTarget())
                //.afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                .waitSeconds(0.5)
                //.afterTime(0.0, () -> armSubsystem.applyNamedPosition("basket high"))
                .setTangent(Math.PI + KEYPOINT_1_TAN)
                .splineToLinearHeading(BASKET_POSE, Math.PI + KEYPOINT_1_TAN)
                //.afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .waitSeconds(0.4)

                // Score center spike mark sample
                //.afterTime(0.5, () -> armSubsystem.applyNamedPosition("intake ground-high"))
                .setTangent(Math.PI / 2)
                .splineToLinearHeading(KEYPOINT_2, Math.PI)
                //.stopAndAdd(armSubsystem.yieldForRotationTarget())
                //.afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                .waitSeconds(0.5)
                //.afterTime(0.0, () -> armSubsystem.applyNamedPosition("basket high"))
                .setTangent(0)
                .splineToLinearHeading(BASKET_POSE, -Math.PI / 2)
                //.afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .waitSeconds(0.4)

                // Score left spike mark sample
                //.afterTime(0.5, () -> armSubsystem.applyNamedPosition("intake ground-high"))
                .setTangent(0)
                .splineToLinearHeading(KEYPOINT_3, Math.PI)
                //.stopAndAdd(armSubsystem.yieldForRotationTarget())
                //.afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.INTAKE))
                .waitSeconds(0.5)
                //.afterTime(0.0, () -> armSubsystem.applyNamedPosition("basket high"))
                .setTangent(0)
                .splineToLinearHeading(BASKET_POSE, Math.PI)
                //.afterTime(0.0, () -> armSubsystem.applyIntakeState(IntakeState.OUTTAKE))
                .waitSeconds(0.4)

                // Park in ascent zone and achieve Level 1 Ascent
                //.afterTime(0.5, () -> armSubsystem.applyNamedPosition("specimen low"))
                .setTangent(Math.PI / 2)
                .splineToLinearHeading(KEYPOINT_4, Math.PI / 2)
                //.stopAndAdd(armSubsystem.yieldForRaiseTarget())
                .setTangent(Math.PI / 2)
                .splineToLinearHeading(KEYPOINT_5, 0)
                //.afterTime(0.0, () -> armSubsystem.setRotationPower(0.0))
                .build());

        return myBot;
    }
}
