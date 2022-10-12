package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoLeftSmall", group = "RedAuto")
public class LeftSmall extends LinearOpMode {

    WebcamExample.SamplePipeline pipeline;
    Robot robot;
    WebcamExample.SamplePipeline.AutoPosition pos;
    @Override
    public void runOpMode() throws InterruptedException {

        while (!isStarted()) {
            pos = pipeline.getAnalysis();
        }

        waitForStart();

        robot = new Robot(this, true);
        robot.initIMU();

        pipeline = new WebcamExample.SamplePipeline();



        //simple sequence to demonstrate the three main autonomous primitives

        //rotate modules to face to the left
        robot.driveController.rotateModules(Vector2d.LEFT, true, 4000, this);

        //drive 20 cm to the left (while facing forward)
        robot.driveController.drive(Vector2d.LEFT, 60.96, 1, this);

        robot.driveController.rotateModules(Vector2d.FORWARD, true, 4000, this);

        robot.driveController.drive(Vector2d.FORWARD, 121.92, 1, this);

        robot.driveController.rotateRobot(new Angle(135, Angle.AngleType.NEG_180_TO_180_HEADING), this);

        //TODO: deliver block

        robot.driveController.rotateRobot(Angle.LEFT, this);

        //TODO: delivering multiple

        // left side (location 1)
        if(pos == WebcamExample.SamplePipeline.AutoPosition.CENTER){
            robot.driveController.rotateModules(Vector2d.RIGHT, true, 4000, this);
            robot.driveController.drive(Vector2d.FORWARD, 60.96, 1, this);

        }else if(pos == WebcamExample.SamplePipeline.AutoPosition.RIGHT){
            robot.driveController.rotateModules(Vector2d.RIGHT, true, 4000, this);
            robot.driveController.drive(Vector2d.FORWARD, 2* 60.96, 1, this);

        }
    }
}
