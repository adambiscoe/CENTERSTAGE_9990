package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);
        Pose2d startPose = new Pose2d(10, -61, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(44.504408918873644, 44.504408918873644, Math.toRadians(180), Math.toRadians(180), -13.11)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .forward(14)
                                .splineToLinearHeading(new Pose2d(10, -33, Math.toRadians(180)), Math.toRadians(0))
                                .back(4)
                                //.splineToLinearHeading(new Pose2d(36, -36.5, Math.toRadians(357)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(44, -33, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-65, 10, Math.toRadians(0)), Math.toRadians(180))
                                .build());

        //MODEL
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(44.504408918873644, 44.504408918873644, Math.toRadians(180), Math.toRadians(180), -13.11)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-31, -61, Math.toRadians(90)))
                                .forward(14)
                                .splineToLinearHeading(new Pose2d(-29, -28, Math.toRadians(0)), Math.toRadians(0))
                                .back(14)
                                .lineToLinearHeading(new Pose2d(-31, -57.5, Math.toRadians(9)))
                                //.lineTo(new Vector2d(20, -57.5))
                                .forward(52)
                                .splineToSplineHeading(new Pose2d(52, -41, Math.toRadians(0)), Math.toRadians(0))
                                .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
               // .addEntity(myBot)
                .addEntity(mySecondBot)
                .start();
    }
}