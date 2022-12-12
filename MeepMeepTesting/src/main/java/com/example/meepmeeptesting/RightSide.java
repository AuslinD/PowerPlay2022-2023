package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RightSide {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -60, Math.toRadians(90)))
                                .forward(25)
                                .turn(Math.toRadians(-45))
                                .forward(5)
                                //drop cone//
                                .forward(-5)
                                .turn(Math.toRadians(45))
                                .forward(23)
                                .turn(Math.toRadians(-90))
                                .forward(22)
                                //get cone1//
                                .forward(-22)
                                .turn(Math.toRadians(135))
                                .forward(5)
                                //drop cone//
                                .forward(-5)
                                .turn(Math.toRadians(-135))
                                .forward(22)
                                //get cone2//
                                .forward(-22)
                                .turn(Math.toRadians(135))
                                .forward(5)
                                //drop cone//
                                .forward(-5)
                                .turn(Math.toRadians(-135))
                                .forward(22)
                                //get cone3//
                                .forward(-22)
                                .turn(Math.toRadians(135))
                                .forward(5)
                                //drop cone//
                                .forward(-5)
                                .turn(Math.toRadians(-135))
                                .forward(22)
                                //get cone4//
                                .forward(-22)
                                .turn(Math.toRadians(135))
                                .forward(5)
                                //drop cone//
                                .forward(-5)
                                .turn(Math.toRadians(-135))
                                .forward(22)
                                //get cone5//
                                .forward(-22)
                                .turn(Math.toRadians(135))
                                .forward(5)
                                //drop cone//
                                .forward(-5)
                                .turn(Math.toRadians(-135))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}