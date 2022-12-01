package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous(name = "Right Auto", group = "Auto")

public class RightAuto extends LinearOpMode{
    Robot robot;
    Drivetrain drivetrain;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family

    AprilTagDetection tagOfInterest = null;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        robot.manip.clawGrab();
        AUto auto = new AUto(robot);
        Drivetrain drivetrain = auto.robot.getDrivetrain();
        robot.manip.clawGrab();
        char pos = ' ';
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == 9) {
                        pos = 'L';
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    if(tag.id == 12) {
                        pos = 'C';
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    if(tag.id == 15) {
                        pos = 'R';
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }
        telemetry.update();
        if(pos == ' '){
            pos = 'R';
        }
        telemetry.addData("pos", pos);
        telemetry.update();
        robot.manip.setPosition(450);
        sleep(300);
        auto.drive(1315,5,this);
        auto.turn(44, 3, this);
        //start to score//
        auto.PIDDrive(400,0.03,0, 0.001,5,this);
        robot.manip.setPosition(750);
        auto.drive(155, 2, this);
        sleep(750);

        robot.manip.setPower(0.5);
        robot.manip.setPosition(400);
        sleep(500);
        robot.manip.clawRelease();
        //robot releases//
        sleep(500);
        auto.drive(-400,3,this);
        auto.turn(0,2,this);

        //beginning of cycle//
        auto.drive(1250,3,this);
        auto.turn(-80, 3, this);
        int cyclepos = 220;
        robot.manip.setPosition(cyclepos);
        auto.drive(1500, 3, this);//tgis
        sleep(100);
        robot.manip.clawGrab();
        sleep(700);
        robot.manip.setPosition(500);
        sleep(200);
        auto.drive(-360,5,this);
        auto.turn(145, 3, this);//this
        sleep(500);
        auto.drive(200,2,this);
        //release was here before
        //auto.turn(180,2,this);
        robot.manip.setPosition(200);
        sleep(100);
        robot.manip.clawRelease();
        sleep(200);
        auto.drive(-150,3,this);
        robot.manip.setPosition(0)  ;
        //auto.drive(200,2,this);
        /*cyclepos =- 20;
        robot.manip.setPosition(cyclepos);*/




        telemetry.addData("position", pos);
        telemetry.update();
        switch(pos) {
            case 'L':
                auto.turn(90, 3, this);
                auto.drive(2400, 3,this);
                break;
            case 'C':
                auto.turn(90, 2,this);
                auto.drive(1100, 3, this);
                break;
            case 'R':
                auto.turn(-65,5,this);
                auto.drive(250,3,this);
                break;
            default:
                auto.turn(-65, 5, this);
                auto.drive(250, 3,this);
                sleep(200);
        }
        robot.manip.setPower(0.5);
        robot.manip.setPosition(0);



    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}

