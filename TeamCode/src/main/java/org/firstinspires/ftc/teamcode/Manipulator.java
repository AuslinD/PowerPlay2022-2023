package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Manipulator {
    private LinearOpMode linear_OpMode;
    private OpMode iterative_OpMode;

    double GRAB = 1;
    double UNGRAB = -1;

    private boolean grabEnabled = false;

    private DcMotor leftLift;
    private DcMotor rightLift;
    private int leftLiftTarget;
    private int rightLiftTarget;

    Servo claw;
    private ElapsedTime clawTimer = new ElapsedTime();
    double clawPrevTime;

    public Manipulator(LinearOpMode opMode) {
        this.linear_OpMode = opMode;

        leftLiftTarget = 0;
        rightLiftTarget = 0;

        leftLift = opMode.hardwareMap.get(DcMotorEx.class,  "liftLeft");
        rightLift = opMode.hardwareMap.get(DcMotorEx.class,  "liftRight");
        claw = opMode.hardwareMap.get(Servo.class, "claw");


        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        linear_OpMode.telemetry.addLine("Manipulator Init Completed - Linear");
        linear_OpMode.telemetry.update();
    }

    public Manipulator(OpMode opMode) {
        this.iterative_OpMode = opMode;

        leftLiftTarget = 0;
        rightLiftTarget = 0;

        leftLift = this.iterative_OpMode.hardwareMap.get(DcMotorEx.class,  "liftLeft");
        rightLift = this.iterative_OpMode.hardwareMap.get(DcMotorEx.class,  "liftRight");
        claw = opMode.hardwareMap.get(Servo.class, "claw");

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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

    public void teleOpControls(Gamepad gamepad2){
        if(gamepad2.a){
            clawGrab();
        }
        if(gamepad2.b){
            clawRelease();
        }
        if(Math.abs(gamepad2.left_stick_y) > 0.1){
            leftLiftTarget += (int)(gamepad2.left_stick_y * 10);
            leftLift.setPower(gamepad2.left_stick_y * .5);
            rightLift.setPower(gamepad2.left_stick_y * .5);


        }
        else{
            rightLift.setPower(0);
            leftLift.setPower(0);

        }
        iterative_OpMode.telemetry.addData("left target", leftLiftTarget);
        iterative_OpMode.telemetry.addData("leftLiftEncoder", leftLift.getCurrentPosition());
        iterative_OpMode.telemetry.addData("rightLiftEncoder", rightLift.getCurrentPosition());

    }

    public double liftPower(double speed){
        double k_P = 0.004;

        double error = leftLiftTarget - leftLift.getCurrentPosition();
        double liftSpeed = speed * k_P * error;
        if(liftSpeed > 1){
            liftSpeed = 1;
        }
        return liftSpeed;
    }
}