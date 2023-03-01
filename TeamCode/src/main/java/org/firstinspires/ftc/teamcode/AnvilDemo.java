package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import ftc.rogue.blacksmith.Anvil;
import ftc.rogue.blacksmith.units.GlobalUnits;

@Autonomous(name = "Anvil Auto", group = "auto")
public class AnvilDemo extends AnvilTest{
    public AnvilDemo(){
        startPose = GlobalUnits.pos(-35.5, 67.75, -90);
    }
    @Override
    protected Anvil mainTraj(Pose2d startPose) {
        return Anvil.forgeTrajectory(drive, startPose)
                .addTemporalMarker(() -> manipulator.setPosition(900))
                .lineToSplineHeading(-35.5, 12, -45)
                .forward(2.5)
                .addTemporalMarker(() -> manipulator.setPosition(2000))
                .waitTime(1)
                .addTemporalMarker(() -> {
                    manipulator.clawRelease();
                    manipulator.setPosition(380);
                })
                .back(2.5)
                .lineToLinearHeading(-59, 12, -180)
                .addTemporalMarker(() -> manipulator.clawGrab())
                .waitTime(.4)
                .addTemporalMarker(() -> manipulator.setPosition(900))
                .thenRun(this::parkTraj);
    }
    private Anvil parkTraj(Pose2d startPose){
        //add in logic might not work

        return Anvil.forgeTrajectory(drive, startPose, (instance) -> {
            if(pos == 'L'){
                instance.lineToSplineHeading(-15.5, 12, 90);
            }
            else if(pos == 'C'){
                instance.lineToSplineHeading(-35.5, 12, 90);
            }
            else{
                instance.lineToLinearHeading(-59, 12, 90);
            }
            return instance;
        });
    }
}
