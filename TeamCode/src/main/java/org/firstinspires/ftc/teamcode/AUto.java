package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

    public void drive(double distance, double speed, int timeout, LinearOpMode opMode){

        DcMotor wheel = drive.getFl();
        double initPos = wheel.getCurrentPosition();
        ElapsedTime runtime = new ElapsedTime();
        while (opMode.opModeIsActive() && runtime.seconds() < timeout && Math.abs(wheel.getCurrentPosition() - initPos) < Math.abs(distance)){
            drive.setMotorPowers(-speed,speed,speed,-speed);
        }
        drive.setAllMotors(0);
    }

    public void turn(double angle, double speed, int timeout, LinearOpMode opMode){
        DcMotor wheel = drive.getFl();
        double initPos = robot.imu.getAngularOrientation().firstAngle;
        ElapsedTime runtime = new ElapsedTime();
        while (opMode.opModeIsActive()&& runtime.seconds() < timeout && Math.abs(robot.imu.getAngularOrientation().firstAngle - initPos) < Math.abs(angle)){
            drive.setMotorPowers(-speed,speed,-speed,speed);
        }
    }
}
