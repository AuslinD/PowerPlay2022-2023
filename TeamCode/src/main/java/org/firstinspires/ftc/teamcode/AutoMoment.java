package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoMoment {
    static Robot robot;

    public static void drive(double distance, double speed, boolean isForward, int timeout){

        DcMotor wheel = robot.drivetrain.getFl();
        double initPos = wheel.getCurrentPosition();
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < timeout && Math.abs(wheel.getCurrentPosition() - initPos) < distance){
            if(isForward)
                robot.drivetrain.setAllMotors(speed);
            else
                robot.drivetrain.setAllMotors(-speed);
        }

        robot.drivetrain.setAllMotors(0);
    }
}
