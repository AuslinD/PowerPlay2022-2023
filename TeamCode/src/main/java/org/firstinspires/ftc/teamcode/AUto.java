package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

 public class AUto{
    Manipulator manipulator;
    private Drivetrain drive;
    Robot robot;
    private PID pid = new PID(0.025,0.008,0,180);

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
        double startPos = robot.imu.getAngularOrientation().firstAngle;
        ElapsedTime runtime = new ElapsedTime();
        opMode.telemetry.addData("loop",  Math.abs(robot.imu.getAngularOrientation().firstAngle - angle));
        opMode.telemetry.update();
        while (opMode.opModeIsActive() && runtime.seconds() < timeout && Math.abs(robot.imu.getAngularOrientation().firstAngle - Math.abs(angle)) > 5){
            double newPower = pid.loop(robot.imu.getAngularOrientation().firstAngle, runtime.seconds());
            opMode.telemetry.addData("newPower", newPower);
            opMode.telemetry.update();
            drive.setMotorPowers(newPower,newPower,-newPower,-newPower);
            opMode.telemetry.addData("Motor spin", newPower);
            opMode.telemetry.addData("angle", robot.imu.getAngularOrientation().firstAngle);
        }

        drive.setMotorPowers(0,0,0,0);
    }
}
