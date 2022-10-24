package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "SimpleSwerve", group = "Teleop")
public class SimpleSwerve extends OpMode {

    public DcMotor fl;
    public DcMotor bl;
    public DcMotor fr;
    public DcMotor br;

    FtcDashboard dash;
    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;


    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("leftTopMotor");
        bl = hardwareMap.dcMotor.get("leftBottomMotor");
        fr = hardwareMap.dcMotor.get("rightTopMotor");
        br = hardwareMap.dcMotor.get("rightBottomMotor");
        dash = FtcDashboard.getInstance();

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void loop(){
        double left = gamepad1.left_stick_y;
        double right = gamepad1.right_stick_y;

        if((Math.abs(gamepad1.left_stick_y) > .001)){
            fl.setPower(left);
            bl.setPower(-left);
        }else{
            fl.setPower(0);
            bl.setPower(0);
        }
        if((Math.abs(gamepad1.right_stick_y) > .001)){
            fr.setPower(right);
            br.setPower(-right);
        }else{
            fr.setPower(0);
            br.setPower(0);
        }
    }
}
