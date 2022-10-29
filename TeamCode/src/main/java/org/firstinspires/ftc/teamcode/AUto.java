package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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

    public void drive(double distance, double speed, int timeout){

        DcMotor wheel = drive.getFl();
        double initPos = wheel.getCurrentPosition();
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < timeout && Math.abs(wheel.getCurrentPosition() - initPos) < Math.abs(distance)){
                drive.setAllMotors(speed);
                telemetry.addData("Motor power ", drive.getFl().getPower());
        }

        drive.setAllMotors(0);
    }

    public void turn(double angle, double speed, int timeout){
        DcMotor wheel = drive.getFl();
        double initPos = robot.imu.getAngularOrientation().firstAngle;
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < timeout && Math.abs(robot.imu.getAngularOrientation().firstAngle - initPos) < Math.abs(angle)){
            telemetry.addData("turning speed", drive.getFl().getPower());
            drive.setMotorPowers(-speed,speed,-speed,speed);
        }
    }
}
