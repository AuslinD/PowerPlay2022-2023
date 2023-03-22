package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import ftc.rogue.blacksmith.Anvil;
import ftc.rogue.blacksmith.units.GlobalUnits;

@Autonomous(name = "Anvil Auto", group = "auto")
public class AnvilDemo extends AnvilTest{
    public AnvilDemo(){
        startPose = GlobalUnits.pos(-35.5, 65.5, -90);
    }
    @Override
    protected Anvil mainTraj(Pose2d startPose) {
        return Anvil.forgeTrajectory(drive, startPose)
                .addTemporalMarker(() -> manipulator.setPosition(900))
                .lineTo(-33.5, 34)
                .addTemporalMarker(() -> manipulator.setPosition(2050))
                .lineToLinearHeading(-29, 19, -45)
                .lineTo(-28, 10.25)
                .waitTime(.5)
                .addTemporalMarker(() -> {
                    manipulator.setPosition(850);
                })
                .waitTime(.1)
                .addTemporalMarker(() -> {
                    manipulator.clawRelease();
                })
                .lineTo(-35.5, 20)
                .addTemporalMarker(() ->manipulator.setPosition(315))
                .turn(-150)
                .lineTo(-62.5, 16.8)

                .addTemporalMarker(() -> manipulator.clawGrab())
                .waitTime(.5)
                .addTemporalMarker(() -> manipulator.setPosition(850))

                .lineTo(-35.5, 22)
                .turn(170)
                .lineTo(-32, 11.9)
                .addTemporalMarker(() -> manipulator.setPosition(1950))
                .lineTo(-30.5, 10.4)
                .waitTime(1)
                .addTemporalMarker(() -> {
                    manipulator.clawRelease();
                    manipulator.setPosition(380);
                })
                .waitTime(0.5)
                .lineTo(-35.5, 27)
                .thenRun(this::parkTraj);
    }
    private Anvil parkTraj(Pose2d startPose){
        //add in logic might not work

        return Anvil.forgeTrajectory(drive, startPose, (instance) -> {
            if(pos == 'L'){
                instance.lineToSplineHeading(-13.5, 22, 0);
            }
            else if(pos == 'C'){
                instance.lineToSplineHeading(-35.5, 24, 0);
            }
            else{
                instance.lineToLinearHeading(-57, 26, 0);
            }
            return instance;
        });
    }
}
