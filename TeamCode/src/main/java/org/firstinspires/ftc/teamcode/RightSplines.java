package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@Autonomous(name = "Right Splines", group = "Auto")
public class RightSplines extends LinearOpMode {
    Manipulator manipulator;
    Drivetrain drivetrain;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    int CONE_HEIGHT = 60;
    int angleOffset = 5;
    int xOffSet = 0;
    int yOffSet = 0;
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


        manipulator = new Manipulator(this);
        manipulator.clawGrab();
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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory forward = drive.trajectoryBuilder(new Pose2d(-35.5, 67.75, Math.toRadians(-90)))
                .forward(48)
                .build();
        Trajectory start = drive.trajectoryBuilder(forward.end())
                .lineToSplineHeading(new Pose2d(-30, 10.25, Math.toRadians(-45 - angleOffset)))
                .addDisplacementMarker(() ->{
                    angleOffset += 2;
                })
                .build();
        Trajectory deliverPreStack = drive.trajectoryBuilder(start.end())
                .forward(5)
                .addDisplacementMarker( () ->{
                    manipulator.setPosition((int) manipulator.TOP_BOUND+000);

                })
                .forward(4)
                .addTemporalMarker(5,() ->{
                    manipulator.setPosition((int) manipulator.TOP_BOUND+700);
                    //manip.stop waits for milliseconds
                    manipulator.stop(500);

                })
                .addTemporalMarker(5.5,() ->{
                    manipulator.clawRelease();
                })
                .build();
        Trajectory backPre = drive.trajectoryBuilder(deliverPreStack.end())
                .back(9)
                .addDisplacementMarker(() -> {
                    manipulator.setPosition((int) manipulator.LOW_BOUND - 8 * CONE_HEIGHT);
                })
                .build();
        Trajectory toFiveStack = drive.trajectoryBuilder(backPre.end())
                .lineToSplineHeading(new Pose2d(-48 + xOffSet, 10.05 + yOffSet, Math.toRadians(-180)))
                .addDisplacementMarker(() ->{
                    manipulator.clawGrab();
                    manipulator.setPosition((int) manipulator.TOP_BOUND / 3);
                    manipulator.stop(1000);
                })
                .build();

        Trajectory firstHigh = drive.trajectoryBuilder(toFiveStack.end())
                .lineToLinearHeading(new Pose2d(-35 + xOffSet, 11.35 + yOffSet, Math.toRadians(-45 - angleOffset)))
                .addDisplacementMarker(() ->{
                    angleOffset += 1;
                })
                .build();
        Trajectory firstDeliver = drive.trajectoryBuilder(firstHigh.end())
                .forward(5)
                .addDisplacementMarker(() ->{
                    manipulator.setPosition((int) manipulator.TOP_BOUND);
                })
                .forward(2)
                .addDisplacementMarker(() ->{
                    manipulator.setPosition((int) manipulator.TOP_BOUND + 400);
                    manipulator.clawRelease();
                    manipulator.stop(1000);
                    //sleep(100);

                })
                .build();
        Trajectory firstBack = drive.trajectoryBuilder(firstDeliver.end())
                .back(9)
                .addDisplacementMarker(() -> {
                    manipulator.setPosition((int) manipulator.LOW_BOUND - 8 * CONE_HEIGHT);
                })
                .build();

        Trajectory secondFiveStack = drive.trajectoryBuilder(firstBack.end())
                .lineToLinearHeading(new Pose2d(-53 + xOffSet, 10.05 + yOffSet, Math.toRadians(-180)))
                .addDisplacementMarker(() ->{
                    manipulator.clawGrab();
                    manipulator.setPosition((int) manipulator.TOP_BOUND / 3);
                    manipulator.stop(1000);
                })
                .build();
        Trajectory secondHigh = drive.trajectoryBuilder(secondFiveStack.end())
                .lineToSplineHeading(new Pose2d(-30 + xOffSet, 13.25 + yOffSet, Math.toRadians(-45 - angleOffset)))
                .addDisplacementMarker(() ->{
                    angleOffset += 1;
                })
                .build();
        Trajectory secondDeliver = drive.trajectoryBuilder(secondHigh.end())
                .forward(5)
                .addTemporalMarker(0, () ->{
                    manipulator.setPosition((int) manipulator.TOP_BOUND);
                })
                .forward(2)
                .addDisplacementMarker(() ->{
                    manipulator.setPosition((int) manipulator.TOP_BOUND + 100);
                    manipulator.clawRelease();
                    manipulator.stop(500);
                    //sleep(100);

                })
                .build();
        Trajectory secondBack = drive.trajectoryBuilder(secondDeliver.end())
                .back(9)
                .addDisplacementMarker(() -> {
                    manipulator.setPosition((int) manipulator.LOW_BOUND - 6 * CONE_HEIGHT);
                })
                .build();

        Trajectory thirdFiveStack = drive.trajectoryBuilder(secondBack.end())
                .lineToLinearHeading(new Pose2d(-58 + xOffSet, 12 + yOffSet, Math.toRadians(-180)))
                .addDisplacementMarker(() ->{
                    manipulator.clawGrab();
                    manipulator.setPosition((int)manipulator.TOP_BOUND / 3);
                    manipulator.stop(1000);
                })
                .build();
        Trajectory thirdHigh = drive.trajectoryBuilder(thirdFiveStack.end())
                .lineToSplineHeading(new Pose2d(-30 + xOffSet, 13.25 + yOffSet, Math.toRadians(-45 - angleOffset)))
                .build();
        Trajectory thirdDeliver = drive.trajectoryBuilder(thirdHigh.end())
                .forward(5)
                .addTemporalMarker(0, () ->{
                    manipulator.setPosition((int) manipulator.TOP_BOUND);
                })
                .forward(2)
                .addDisplacementMarker(() ->{
                    manipulator.setPosition((int) manipulator.TOP_BOUND + 200);
                    manipulator.clawRelease();
                    manipulator.stop(500);
                    //sleep(100);

                })
                .build();
        Trajectory thirdBack = drive.trajectoryBuilder(thirdDeliver.end())
                .back(9)
                .addDisplacementMarker(() -> {
                    manipulator.setPosition((int) manipulator.LOW_BOUND - 4 * CONE_HEIGHT);
                })
                .build();

        Trajectory fourFiveStack = drive.trajectoryBuilder(thirdBack.end())
                .lineToLinearHeading(new Pose2d(-59, 12, Math.toRadians(-180 + angleOffset)))
                .addDisplacementMarker(() ->{
                    manipulator.clawGrab();
                    manipulator.stop(1000);
                })
                .build();
        Trajectory fourHigh = drive.trajectoryBuilder(fourFiveStack.end())
                .lineToSplineHeading(new Pose2d(-30, 13.25, Math.toRadians(-45 - angleOffset)))
                .build();
        Trajectory fourDeliver = drive.trajectoryBuilder(fourHigh.end())
                .forward(5)
                .addTemporalMarker(0, () ->{
                    manipulator.setPosition((int) manipulator.TOP_BOUND);
                })
                .forward(2)
                .addDisplacementMarker(() ->{
                    manipulator.setPosition((int) manipulator.TOP_BOUND - 400);
                    manipulator.clawRelease();
                    manipulator.stop(500);
                    manipulator.setPosition((int) manipulator.LOW_BOUND);
                })
                .build();
        Trajectory fourBack = drive.trajectoryBuilder(fourDeliver.end())
                .back(7)
                .addDisplacementMarker(() -> {
                    manipulator.setPosition((int) manipulator.LOW_BOUND - 8 * CONE_HEIGHT);
                })
                .build();

        Trajectory fiveFiveStack = drive.trajectoryBuilder(fourBack.end())
                .lineToLinearHeading(new Pose2d(-59, 12, Math.toRadians(-180 + angleOffset)))
                .addDisplacementMarker(() ->{
                    manipulator.clawGrab();
                    manipulator.stop(1000);
                })
                .build();
        Trajectory fiveHigh = drive.trajectoryBuilder(fiveFiveStack.end())
                .lineToSplineHeading(new Pose2d(-29, 12.25, Math.toRadians(-45 - angleOffset)))
                .build();
        Trajectory fiveDeliver = drive.trajectoryBuilder(fiveHigh.end())
                .forward(5)
                .addTemporalMarker(0, () ->{
                    manipulator.setPosition((int) manipulator.TOP_BOUND);
                })
                .forward(2)
                .addDisplacementMarker(() ->{
                    manipulator.setPosition((int) manipulator.TOP_BOUND - 400);
                    manipulator.clawRelease();
                    manipulator.setPosition((int) manipulator.LOW_BOUND);
                })
                .build();
        Trajectory fiveBack = drive.trajectoryBuilder(fiveDeliver.end())
                .back(7)
                .build();
        //TODO: CHANGE BACK THE START POSE
        Trajectory rightPark = drive.trajectoryBuilder(thirdBack.end())
                .lineToLinearHeading(new Pose2d(-58, 12, Math.toRadians(180)))
                .build();
        Trajectory midPark = drive.trajectoryBuilder(thirdBack.end())
                .lineToLinearHeading(new Pose2d(-31, 12, Math.toRadians(180)))
                .build();
        Trajectory leftPark = drive.trajectoryBuilder(thirdBack.end())
                .lineToLinearHeading(new Pose2d(-14, 12, Math.toRadians(180)))
                .build();
        waitForStart();

        if(isStopRequested()) return;


        drive.setPoseEstimate(new Pose2d(-35.5, 64.75, Math.toRadians(-90)));
        drive.followTrajectory(start);
        drive.followTrajectory(deliverPreStack);
        drive.followTrajectory(backPre);

        drive.followTrajectory(toFiveStack);
        drive.followTrajectory(firstHigh);
        drive.followTrajectory(firstDeliver);
        drive.followTrajectory(firstBack);

        drive.followTrajectory(secondFiveStack);
        drive.followTrajectory(secondHigh);
        drive.followTrajectory(secondDeliver);
        drive.followTrajectory(secondBack);

        drive.followTrajectory(thirdFiveStack);
        drive.followTrajectory(thirdHigh);
        drive.followTrajectory(thirdDeliver);
        drive.followTrajectory(thirdBack);
        /*

        drive.followTrajectory(fourFiveStack);
        drive.followTrajectory(fourHigh);
        drive.followTrajectory(fourDeliver);
        drive.followTrajectory(fourBack);

        drive.followTrajectory(fiveFiveStack);
        drive.followTrajectory(fiveHigh);
        drive.followTrajectory(fiveDeliver);
        drive.followTrajectory(fiveBack);
         */

        //TODO: implement change park
        telemetry.update();
        switch(pos) {
            case 'L':
                drive.followTrajectory(leftPark);
                break;
            case 'C':
                drive.followTrajectory(midPark);
                break;
            case 'R':
                drive.followTrajectory(rightPark);
                break;
            default:
                drive.followTrajectory(rightPark);
        }
        manipulator.setPosition(0);

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
