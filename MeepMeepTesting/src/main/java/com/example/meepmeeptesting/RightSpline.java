package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RightSpline {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14.5, 13.5)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35.5, -64.75, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(34, -10.25, Math.toRadians(135)))
                                .forward(5)
                                .forward(2)
                                .back(7)
                                .lineToSplineHeading(new Pose2d(59, -12, Math.toRadians(0)))

                                .lineToLinearHeading(new Pose2d(34, -10.25, Math.toRadians(135)))
                                .forward(5)
                                .forward(2)
                                .back(7)
                                .lineToLinearHeading(new Pose2d(59, -12, Math.toRadians(0)))

                                .lineToSplineHeading(new Pose2d(34, -10.25, Math.toRadians(135)))
                                .forward(5)
                                .forward(2)
                                .back(7)
                                .lineToLinearHeading(new Pose2d(59, -12, Math.toRadians(0)))

                                .lineToSplineHeading(new Pose2d(34, -10.25, Math.toRadians(135)))
                                .forward(5)
                                .forward(2)
                                .back(7)
                                .lineToLinearHeading(new Pose2d(59, -12, Math.toRadians(0)))

                                .lineToSplineHeading(new Pose2d(34, -10.25, Math.toRadians(135)))
                                .forward(5)
                                .forward(2)
                                .back(7)
                                .lineToLinearHeading(new Pose2d(59, -12, Math.toRadians(0)))

                                .lineToSplineHeading(new Pose2d(34, -10.25, Math.toRadians(135)))
                                .forward(5)
                                .forward(2)
                                .back(7)
                                .lineToLinearHeading(new Pose2d(58, -12, Math.toRadians(180)))



                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
