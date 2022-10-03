package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@TeleOp(name = "test teleop2",group = "test teleop2")
public class megaman extends OpMode {
    public DcMotor fl;
    public DcMotor bl;
    public DcMotor fr;
    public DcMotor br;

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;




    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("fl");
        bl = hardwareMap.dcMotor.get("bl");
        fr = hardwareMap.dcMotor.get("fr");
        br = hardwareMap.dcMotor.get("br");


    }

    @Override
    public void loop() {
        double forward = gamepad1.left_stick_y;
        double sideways = -gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double frp;
        double flp;
        double blp;
        double brp;

        frp = -forward;
        flp = -forward;
        blp = -forward;
        brp = -forward;

        frp += sideways;
        flp -= sideways;
        brp -= sideways;
        blp += sideways;

        frp -= turn;
        flp += turn;
        brp -= turn;
        blp += turn;

    double max = Math.max(Math.max(Math.abs(frp), Math.abs(flp)), Math.max(Math.abs(brp), Math.abs(blp)));

        if(Math.abs(max) >1) {
            frp /= Math.abs(max);
            flp /= Math.abs(max);
            brp /= Math.abs(max);
            blp /= Math.abs(max);
        }
            if(Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1){
                fl.setPower(flp);
                bl.setPower(blp);
                fr.setPower(frp);
                br.setPower(brp);
            }
            else {
                fl.setPower(0);
                bl.setPower(0);
                fr.setPower(0);
                br.setPower(0);

            }

    }
}