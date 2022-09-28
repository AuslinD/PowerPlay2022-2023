package org.firstinspires.ftc.teamcode;

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

    public LinearOpMode opMode;

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;



    public omniman(LinearOpMode opMode) {
        this.opMode = opMode;

    }

    @Override
    public void init() {
        fl = opMode.hardwareMap.dcMotor.get("fl");
        bl = opMode.hardwareMap.dcMotor.get("bl");
        fr = opMode.hardwareMap.dcMotor.get("fr");
        br = opMode.hardwareMap.dcMotor.get("br");

    }

    @Override
    public void loop() {
        if (gamepad1.left_stick_y != 0)  {
            fl.setPower(gamepad1.left_stick_y);
            bl.setPower(gamepad1.left_stick_y);
            fr.setPower(-gamepad1.left_stick_y);
            br.setPower(-gamepad1.left_stick_y);
        }

    }
}
