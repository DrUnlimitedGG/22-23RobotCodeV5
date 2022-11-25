package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class moomoo {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-33.6, 60, 180))
                                .lineToLinearHeading(new Pose2d(-13.2,57.8, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-13.2,13.6, Math.toRadians(-135)))
                                // add marker here to put cone down (preloaded one)
                                .lineToLinearHeading(new Pose2d(-56.8,14, Math.toRadians(-180)))
                                // add marker here to pick up cone
                                .lineToLinearHeading(new Pose2d(-31.2,11.6, Math.toRadians(-63)))
                                // add marker here to put cone down (new one)
                                // then go park lol
                                .splineToSplineHeading(new Pose2d(-36.4,38, Math.toRadians(84)), Math.toRadians(84))
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}