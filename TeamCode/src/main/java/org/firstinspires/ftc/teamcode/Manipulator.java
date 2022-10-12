package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Manipulator {
    private LinearOpMode linear_OpMode;
    private OpMode iterative_OpMode;

    double GRAB = -1;
    double UNGRAB = 1;

    private boolean grabEnabled = false;

    DcMotor lift;
    Servo claw;
    private ElapsedTime clawTimer = new ElapsedTime();
    double clawPrevTime;

    public Manipulator(LinearOpMode opMode) {
        linear_OpMode = opMode;


        lift = opMode.hardwareMap.get(DcMotorEx.class,  "lift");
        claw = opMode.hardwareMap.get(Servo.class, "claw");


        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        linear_OpMode.telemetry.addLine("Manipulator Init Completed - Linear");
        linear_OpMode.telemetry.update();
    }

    public Manipulator(OpMode opMode) {
        iterative_OpMode = opMode;

        lift = opMode.hardwareMap.get(DcMotorEx.class,  "lift");
        claw = opMode.hardwareMap.get(Servo.class, "claw");

        lift.setDirection(DcMotorSimple.Direction.FORWARD);


        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        opMode.telemetry.addLine("Manipulator Init Completed - Iterative");
        opMode.telemetry.update();
    }

    // GRABBY STUFF
    public void clawGrab(){
        claw.setPosition(GRAB);
        grabEnabled = true;
    }

    public void clawRelease(){
        claw.setPosition(UNGRAB);
        grabEnabled = false;
    }

    public void toggleGrabber(){
        if (grabEnabled) {
            clawRelease();
        } else {
            clawGrab();
        }
        grabEnabled = !grabEnabled;
    }

    public void teleOpControls(boolean a){
        if(a){
            clawTimer.reset();
            clawPrevTime = 0;
            if(clawTimer.milliseconds() - clawPrevTime > 200){
                toggleGrabber();
            }
        }
    }
}
