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
        if (gamepad1.left_stick_y != 0)  {
            fl.setPower(gamepad1.left_stick_y);
            bl.setPower(gamepad1.left_stick_y);
            fr.setPower(-gamepad1.left_stick_y);
            br.setPower(-gamepad1.left_stick_y);
        }

    }
}
