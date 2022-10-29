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
        

        }
     }
