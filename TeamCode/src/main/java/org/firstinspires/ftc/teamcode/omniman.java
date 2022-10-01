package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@TeleOp(name = "test teleop",group = "test teleop")
public class omniman extends OpMode {
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
        /**if (gamepad1.left_stick_y != 0)  {
            fl.setPower(gamepad1.left_stick_y);
            bl.setPower(gamepad1.left_stick_y);
            fr.setPower(-gamepad1.left_stick_y);
            br.setPower(-gamepad1.left_stick_y);
        }
        if(gamepad1.right_stick_x !=0) {
            fr.setPower();
            br.setPower();
            fl.setPower();
            bl.setPower();*/
        //sets input value variables
        double turn = gamepad1.left_stick_y;
        double forward = -gamepad1.right_stick_x;

        //gets left and right side motor speeds
        double left = forward - turn;
        double right = forward + turn;

        //checks which motor side is faster and then divides both speeds to get a max of 1 while
        //keeping proportionality
        double max = Math.max(Math.abs(left), Math.abs(right));
        if(max >1){
            left/= max;
            right/= max;
        }
        //sets motor power
        if(Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1){
            fl.setPower(left);
            bl.setPower(left);
            fr.setPower(-right);
            br.setPower(-right);
        }
        else{
            fl.setPower(0);
            bl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
        }
        telemetry.addData("Joystick y", gamepad1.left_stick_y);
        telemetry.addData("Joystick x", gamepad1.right_stick_x);
        telemetry.update();
    }
}
