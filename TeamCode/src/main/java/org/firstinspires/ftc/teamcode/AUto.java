package org.firstinspires.ftc.teamcode;

import android.content.pm.LauncherApps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

 public class AUto{
    Manipulator manipulator;
    private Drivetrain drive;
    Robot robot;
    private PID pid = new PID();

    public AUto(Robot a){
        robot = a;
        drive = robot.getDrivetrain();
    }

    public void drive(double distance, double speed, boolean isForward, int timeout){

        DcMotor wheel = drive.getFl();
        double initPos = wheel.getCurrentPosition();
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < timeout && Math.abs(wheel.getCurrentPosition() - initPos) < distance){
            if(isForward)
                drive.setAllMotors(speed);
            else
                drive.setAllMotors(-speed);
        }

        drive.setAllMotors(0);
    }

    public void turn(double angle, double speed, boolean isLeft, int timeout){
        DcMotor wheel = drive.getFl();
        double initPos = robot.imu.getAngularOrientation().firstAngle;
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < timeout && Math.abs(robot.imu.getAngularOrientation().firstAngle - initPos) < angle){
            if(isLeft)
                drive.setMotorPowers(-speed,speed,-speed,speed);
            else
                drive.setMotorPowers(speed,-speed,speed,-speed);
        }
    }
}
