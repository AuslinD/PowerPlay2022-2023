package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Right Encoder", group = "Auto")
public class RightAutoOdom extends LinearOpMode {
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
        auto.initHeading = robot.imu.getAngularOrientation().firstAngle;
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
        robot.manip.setPosition(-900);
        sleep(100);
        auto.driveOdom(42.25,5,this);//42.25
        auto.toHeading(53, 3, this);

        //start to score//
        auto.PIDDrive(400,0.03,0, 0.001,5,this);
        robot.manip.setPosition(-2200);
        sleep(100);
        auto.PIDDrive(205, 0.007, 0, 0.001, 3, this);
        sleep(50);

        robot.manip.setPower(1);
        robot.manip.setPosition(-1340);
        sleep(100);
        robot.manip.clawRelease();
        //robot releases//
        sleep(50);
        //robot.manip.setPosition(2100);
        //sleep(100);
        auto.drive(-315,3,this);
        int cyclepos = 220;
        robot.manip.setPosition(cyclepos);
        auto.toHeading(-90,2,this);

        //beginning of cycle//
        //auto.drive(1250,3,this);
        //auto.turn(-80, 3, this);

        auto.driveOdom(21, 2, this);//tgis
        //robot.manip.setPosition(-650);
        sleep(100);
        robot.manip.clawGrab();
        sleep(50);
        robot.manip.setPosition(-900);
        sleep(100);
        auto.driveOdom(-21,5,this);
        sleep(50);
        auto.toHeading(45, 3, this);//this
        sleep(50);
        auto.drive(380,2,this);
        //release was here before
        //auto.turn(180,2,this);
        robot.manip.setPosition(-2000);
        auto.PIDDrive(250, 0.007, 0, 0.001, 3, this);
        sleep(50);
        robot.manip.setPosition(-800);
        robot.manip.clawRelease();
        sleep(100);
//cycle 2 electric boogaloo
        /*auto.drive(-300,3,this);
        robot.manip.setPosition(0);
        auto.toHeading(-90,3,this);
        sleep(50);
        robot.manip.setPosition(-800);
        auto.drive(22,5,this);
        //auto.drive(380,3,this);
        sleep(100);
        robot.manip.clawGrab();
        sleep(100);
//not an easter egg and deffinitly not matthew
        robot.manip.setPosition(-1500);
        auto.driveOdom(-23.5,5,this);
        sleep(50);
        auto.toHeading(45,3,this);
        auto.drive(380,2,this);
        robot.manip.setPosition(-1800);
        auto.PIDDrive(250,0.007,0,0.001,3,this);
        robot.manip.setPosition(-1000);
        robot.manip.setPosition(0);




        sleep(100);
        //auto.drive(200,2,this);
        /*cyclepos =- 20;
        robot.manip.setPosition(cyclepos);*/




        telemetry.addData("position", pos);
        telemetry.update();
        switch(pos) {
            case 'L':
                auto.toHeading(90, 3, this);
                auto.drive(1000, 3,this);
                break;
            case 'C':
                auto.toHeading(90, 2,this);
                auto.drive(-100, 3, this);
                break;
            case 'R':
                auto.toHeading(-90,5,this);
                auto.drive(2000,3,this);
                break;
            default:
                auto.toHeading(-90, 5, this);
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
