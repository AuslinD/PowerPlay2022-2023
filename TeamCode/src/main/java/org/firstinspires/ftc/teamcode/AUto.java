package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AUto {
    Robot robot;
    private PID pid = new PID();


    public void drive(double distance, double speed, boolean isForward, int timeout){
        fl.setPower(pid.loop(distance, timeout));
        bl.setPower(pid.loop(distance, timeout));;
        br.setPower(-pid.loop(distance, timeout));;
        fr.setPower(-pid.loop(distance, timeout));;
    }

    public void turn(Angle angle, LinearOpMode LinearOpMode){

    }
}
