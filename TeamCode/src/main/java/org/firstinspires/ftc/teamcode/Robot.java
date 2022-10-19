package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Robot {
    Drivetrain drivetrain;
    //Manipulator manip;

    private OpMode teleOp;
    private LinearOpMode auto;

    public Robot(LinearOpMode opMode){
        auto = opMode;

        drivetrain = new Drivetrain(opMode);
        //manip = new Manipulator(opMode);


        opMode.telemetry.addLine("Crabtrain Init Completed - Linear");
        opMode.telemetry.update();
    }

    public Robot(OpMode opMode) {
        teleOp = opMode;

        drivetrain = new Drivetrain(opMode);
        //manip = new Manipulator(opMode);

        opMode.telemetry.addLine("Crabtrain Init Completed - Iterative");
        opMode.telemetry.update();
    }


}
