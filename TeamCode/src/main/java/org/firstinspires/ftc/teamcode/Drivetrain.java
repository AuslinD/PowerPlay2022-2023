package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


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

        opMode.telemetry.addLine("Drivetrain Init Completed - Iterative");
        opMode.telemetry.update();
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

    public void setAllMotors(double power) {
        fr.setPower(power);
        fl.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }


}
