package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Mecanum auto test", group = "Auto")

public class TestAutoMecanum extends LinearOpMode{
     Robot robot;
     Drivetrain drivetrain;

     static OpenCvWebcam webcam;
     WebcamExample.SamplePipeline pipeline;
     WebcamExample.SamplePipeline.AutoPosition pos;


     @Override
     public void runOpMode() throws InterruptedException {
          robot = new Robot(this);
          AUto no = new AUto(robot);
          Drivetrain drivetrain = no.robot.getDrivetrain();
          while (!isStarted()) {
               telemetry.addData("Turning Angle 1", robot.imu.getAngularOrientation().firstAngle);
               telemetry.addData("Turning Angle 2", robot.imu.getAngularOrientation().secondAngle);
               telemetry.addData("Turning Angle 3", robot.imu.getAngularOrientation().thirdAngle);
               telemetry.update();
          }
          waitForStart();
          no.turn(180, 0.5, 5, this);

          pipeline = new WebcamExample.SamplePipeline();

          int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
          webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
          webcam.setPipeline(pipeline);
          webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
          webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
          {
               @Override
               public void onOpened()
               {
                    /*
                     * Tell the webcam to start streaming images to us! Note that you must make sure
                     * the resolution you specify is supported by the camera. If it is not, an exception
                     * will be thrown.
                     *
                     * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                     * supports streaming from the webcam in the uncompressed YUV image format. This means
                     * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                     * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                     *
                     * Also, we specify the rotation that the webcam is used in. This is so that the image
                     * from the camera sensor can be rotated such that it is always displayed with the image upright.
                     * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                     * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                     * away from the user.
                     */
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
               }

               @Override
               public void onError(int errorCode)
               {
                    /*
                     * This will be called if the camera could not be opened
                     */
               }
          });
          while (!isStarted()) {
               telemetry.addData("pos", pipeline.getAnalysis());
               telemetry.update();

               pos = pipeline.getAnalysis();
          }

          telemetry.addLine("Waiting for start");
          telemetry.update();
          waitForStart();
          if (pos == WebcamExample.SamplePipeline.AutoPosition.LEFT) {
               telemetry.addLine("detected pos1");

          }
          else if(pos == WebcamExample.SamplePipeline.AutoPosition.CENTER) {
               telemetry.addLine("deteccted pos2");


          }
          else {
               telemetry.addLine("deteccted pos3");

          }
          telemetry.update();




    }
}
