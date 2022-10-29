package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

     @Autonomous(name = "Mecanum auto test", group = "Auto")

public class TestAutoMecanum extends LinearOpMode{
        Robot robot;
        AUto no = new AUto(robot);
        Drivetrain drivetrain = no.robot.getDrivetrain();
         @Override
         public void runOpMode() throws InterruptedException {
            no.drive(2400,1, true,3);
            no.turn(360, 0.5, true, 5);
            telemetry.addData("Turning Angle 1", robot.imu.getAngularOrientation().firstAngle);
            telemetry.addData("Turning Angle 2", robot.imu.getAngularOrientation().secondAngle);
            telemetry.addData("Turning Angle 3", robot.imu.getAngularOrientation().thirdAngle);
        }
     }
