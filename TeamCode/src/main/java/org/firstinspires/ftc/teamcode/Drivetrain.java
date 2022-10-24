package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;


public class Drivetrain {
    private DcMotor fr, fl, br, bl;
    private LinearOpMode linearOpMode;
    private OpMode iterativeOpMode;

    public Drivetrain(LinearOpMode opMode){
        this.linearOpMode = opMode;

        fr = this.linearOpMode.hardwareMap.get(DcMotorEx.class, "fr");
        fl = this.linearOpMode.hardwareMap.get(DcMotorEx.class, "fl");
        bl = this.linearOpMode.hardwareMap.get(DcMotorEx.class, "bl");
        br = this.linearOpMode.hardwareMap.get(DcMotorEx.class, "br");

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        opMode.telemetry.addLine("Drivetrain Init Completed - Linear");
        opMode.telemetry.update();
    }

    public Drivetrain(OpMode opMode){
        this.iterativeOpMode = opMode;

        fr = this.iterativeOpMode.hardwareMap.get(DcMotorEx.class, "fr");
        fl = this.iterativeOpMode.hardwareMap.get(DcMotorEx.class, "fl");
        bl = this.iterativeOpMode.hardwareMap.get(DcMotorEx.class, "bl");
        br = this.iterativeOpMode.hardwareMap.get(DcMotorEx.class, "br");

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        opMode.telemetry.addLine("Drivetrain Init Completed - Iterative");
        opMode.telemetry.update();
    }

    public void teleOpControls(Gamepad gamepad1){
        double FLP = 0;
        double FRP = 0;
        double BRP = 0;
        double BLP = 0;

        if(Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1){
            FRP += gamepad1.left_stick_y;
            FLP += gamepad1.left_stick_y;
            BRP += gamepad1.left_stick_y;
            BLP += gamepad1.left_stick_y;

            FRP += -gamepad1.left_stick_x * 1.1;
            FLP += -gamepad1.left_stick_x * 1.1;
            BLP += -gamepad1.left_stick_x * 1.1;
            BRP += -gamepad1.left_stick_x * 1.1;

            FRP += gamepad1.right_stick_x;
            FLP += gamepad1.right_stick_x;
            BRP += gamepad1.right_stick_x;
            BLP += gamepad1.right_stick_x;

            double max = Math.max(Math.max(Math.abs(FRP), Math.abs(FLP)), Math.max(Math.abs(BRP), Math.abs(BLP)));

            if(Math.abs(max) > 1) {
                FRP /= Math.abs(max);
                FLP /= Math.abs(max);
                BRP /= Math.abs(max);
                BLP /= Math.abs(max);
            }

            if(Math.abs(gamepad1.right_trigger) > 0.1){
                fl.setPower(FLP * .25);
                bl.setPower(BLP * .25);
                fr.setPower(FRP * .25);
                br.setPower(BRP * .25);
            }
            else{
                fl.setPower(FLP);
                bl.setPower(BLP);
                fr.setPower(FRP);
                br.setPower(BRP);
            }


        }
        else{
            fl.setPower(0);
            bl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
        }
    }



    public void setMotorBrake(boolean brake){
        if (brake){
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else {
            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void setMotorPowers(double powerFL, double powerFR, double powerBL, double powerBR) {
        fl.setPower(powerFL);
        fr.setPower(powerFR);
        bl.setPower(powerBL);
        br.setPower(powerBR);
    }

    public DcMotor getFl(){
        return fl;
    }

    public void setAllMotors(double power) {
        fr.setPower(power);
        fl.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }


}
