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
                .addTemporalMarker(() -> manipulator.setPosition(2000))
                .lineToLinearHeading(-27.5, 11.5, -45)
                .waitTime(.5)
                .addTemporalMarker(() -> {
                    manipulator.setPosition(850);
                })
                .waitTime(.1)
                .addTemporalMarker(() -> {
                    manipulator.clawRelease();
                })
                .lineTo(-35.5, 23)
                .addTemporalMarker(() ->manipulator.setPosition(250))
                .turn(-150)
                .lineTo(-65, 19.5)

                .addTemporalMarker(() -> manipulator.clawGrab())
                .waitTime(.5)
                .addTemporalMarker(() -> manipulator.setPosition(850))

                .lineTo(-35.5, 22)
                .turn(170)
                .lineTo(-28.5, 8)
                .addTemporalMarker(() -> manipulator.setPosition(2000))
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
                instance.lineToSplineHeading(-15.5, 22, 0);
            }
            else if(pos == 'C'){
                instance.lineToSplineHeading(-35.5, 22, 0);
            }
            else{
                instance.lineToLinearHeading(-57, 26, 0);
            }
            return instance;
        });
    }
}
