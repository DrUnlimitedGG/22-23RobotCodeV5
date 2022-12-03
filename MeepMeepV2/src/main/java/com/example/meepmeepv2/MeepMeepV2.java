package com.example.meepmeepv2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepV2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-33.69, -68.17, Math.toRadians(90.00)))
                                .lineToConstantHeading(new Vector2d(-11, -63))
                                .lineToLinearHeading(new Pose2d(-10.5, -13, Math.toRadians(133.67)))
                                .forward(10)
                                .strafeLeft(1.125)
                                .lineToLinearHeading(new Pose2d(-13.5, -11, Math.toRadians(185)))
                                .lineToLinearHeading(new Pose2d(-13.5, -63, Math.toRadians(90)))
                                .lineToConstantHeading(new Vector2d(-59.5, -63))
                                .forward(30)

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}