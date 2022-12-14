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

    double GRAB = .623;
    double UNGRAB = 0.47;
    double LIFT_POWER = 0.9;// TODO: change this?
    double LIFT_HOLD_CONSTANT = 0.15;
    double TOP_BOUND = -1100; // because motors are reversed lol
    double LOW_BOUND = 50;
    private double goalEncoder = 0;

    private boolean grabEnabled = false;

    private DcMotor leftLift;
    private DcMotor rightLift;
    private double leftLiftTarget;
    private double rightLiftTarget;

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




        leftLift.setTargetPosition(0);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setPower(LIFT_POWER);


        rightLift.setTargetPosition(0);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setPower(LIFT_POWER);

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

        leftLift.setPower(LIFT_POWER);
        leftLift.setTargetPosition(0);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightLift.setPower(LIFT_POWER);
        rightLift.setTargetPosition(0);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


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

    /*public void toggleGrabber(){
        if (grabEnabled) {
            clawRelease();
        } else {
            clawGrab();
        }
        grabEnabled = !grabEnabled;
    }*/

    public void teleOpControls(Gamepad gamepad2){
        if(gamepad2.a){
            clawGrab();
        }
        if(gamepad2.b){
            clawRelease();
        }
        if(gamepad2.dpad_down){
            leftLiftTarget = 0;
        }
        if(gamepad2.dpad_up){
            leftLiftTarget = -1050;
        }
        if(gamepad2.right_bumper){
            leftLiftTarget = -200;
        }
        /* PREVIOUS LIFT CODE
        if(Math.abs(gamepad2.left_stick_y) > 0.1){
            leftLiftTarget += (int)(gamepad2.left_stick_y * 10);
            leftLift.setPower(gamepad2.left_stick_y * .5);
            rightLift.setPower(gamepad2.left_stick_y * .5);


        }
        else{
            rightLift.setPower(0);
            leftLift.setPower(0);

        }
         */
        if(Math.abs(gamepad2.right_trigger) > 0.1){
            leftLiftTarget += 10 * gamepad2.left_stick_y;
        }
        else{
            // LEFT STICK Y IS REVERSE OF WHAT YOU THINK
            if (gamepad2.left_stick_y < 0){
                if (leftLiftTarget > TOP_BOUND){
                    leftLiftTarget += 10 * gamepad2.left_stick_y;
                }
            }

            if (gamepad2.left_stick_y > 0){
                if (leftLiftTarget < LOW_BOUND){
                    leftLiftTarget += 15 * gamepad2.left_stick_y;
                }
            }


            if(leftLiftTarget > LOW_BOUND){// >< because low bound and top bound are weird DONT CHANGE UNLESS FOR SURE THIS IS ISSUE
                leftLiftTarget = LOW_BOUND;
            }
            else if(leftLiftTarget < TOP_BOUND){
                leftLiftTarget = TOP_BOUND;
            }
        }



        leftLift.setTargetPosition((int)leftLiftTarget);
        rightLift.setTargetPosition((int)leftLiftTarget);


        // TODO: check if this works
        if(leftLift.getCurrentPosition() - leftLiftTarget > 0 && leftLift.getCurrentPosition() - leftLiftTarget < 10){
            leftLift.setPower(LIFT_HOLD_CONSTANT);
            rightLift.setPower(LIFT_HOLD_CONSTANT);
            iterative_OpMode.telemetry.addData("near target position", leftLift.getCurrentPosition() - leftLiftTarget);
        }
        else{
            leftLift.setPower(LIFT_POWER);
            rightLift.setPower(LIFT_POWER);
        }

        iterative_OpMode.telemetry.addData("left target", leftLiftTarget);
        iterative_OpMode.telemetry.addData("leftLiftEncoder", leftLift.getCurrentPosition());
        iterative_OpMode.telemetry.addData("rightLiftEncoder", rightLift.getCurrentPosition());

    }
    public void setPosition(int pos){
        leftLift.setTargetPosition(pos);
        rightLift.setTargetPosition(-pos);


    }

    public void setPower(double power){
        leftLift.setPower(power);
        rightLift.setPower(power);
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
    public boolean isClosed(){
        if (claw.getPosition() == 1){
            return true;
        }
        return false;

    }
    /*
    public void distanceSensor(Gamepad gamepad2, double distance, int timeout) { //figure out how to make it so that when driver presses the release, claw releases for like a few seconds before this code starts again//
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < timeout) {
            if (gamepad2.x) {
                if (distance <= 4.3 && !isClosed()) {
                    clawGrab();

                }
            }
        }
    }    */
}