package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

import ftc.rogue.blacksmith.Anvil;


public class Drivetrain {
    public DcMotor fr, fl, br, bl;
    private LinearOpMode linearOpMode;
    private OpMode iterativeOpMode;

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.748031; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 11.713; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 1.673228; // in; offset of the lateral wheel

    private Encoder leftEncoder, rightEncoder, frontEncoder;
    public static double X_MULTIPLIER = 1.013705869; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.006231168; // Multiplier in the Y direction




    public Drivetrain(LinearOpMode opMode){
        this.linearOpMode = opMode;

        fr = this.linearOpMode.hardwareMap.get(DcMotorEx.class, "fr");
        fl = this.linearOpMode.hardwareMap.get(DcMotorEx.class, "fl");
        bl = this.linearOpMode.hardwareMap.get(DcMotorEx.class, "bl");
        br = this.linearOpMode.hardwareMap.get(DcMotorEx.class, "br");

        leftEncoder = new Encoder(this.linearOpMode.hardwareMap.get(DcMotorEx.class, "leftOdom"));
        //rightEncoder = new Encoder(this.linearOpMode.hardwareMap.get(DcMotorEx.class, "rightOdom"));
        frontEncoder = new Encoder(this.linearOpMode.hardwareMap.get(DcMotorEx.class, "frontOdom"));
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
            FRP -= gamepad1.left_stick_y;
            FLP += gamepad1.left_stick_y;
            BRP += gamepad1.left_stick_y;
            BLP -= gamepad1.left_stick_y;

            FRP += -gamepad1.left_stick_x * 1.1;
            FLP += -gamepad1.left_stick_x * 1.1;
            BLP += -gamepad1.left_stick_x * 1.1;
            BRP += -gamepad1.left_stick_x * 1.1;

            FRP -= gamepad1.right_stick_x;
            FLP -= gamepad1.right_stick_x;
            BRP += gamepad1.right_stick_x;
            BLP += gamepad1.right_stick_x;

            /*
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (-rotY - rotX - rx) / denominator;
            double backLeftPower = (-rotY + rotX - rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

             */

            double max = Math.max(Math.max(Math.abs(FRP), Math.abs(FLP)), Math.max(Math.abs(BRP), Math.abs(BLP)));

            if(Math.abs(max) > 1) {
                FRP /= Math.abs(max);
                FLP /= Math.abs(max);
                BRP /= Math.abs(max);
                BLP /= Math.abs(max);
            }

            if(Math.abs(gamepad1.right_trigger) > 0.1){
                fl.setPower(FLP * .4);
                bl.setPower(BLP * .4);
                fr.setPower(FRP * .4);
                br.setPower(BRP * .4);
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

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;

    }

    @NonNull
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                //encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

}
