package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

 public class AUto{
    Manipulator manipulator;
    private Drivetrain drive;
    Robot robot;


    public AUto(Robot a){
        robot = a;
        drive = robot.getDrivetrain();
    }

    public void drive(double distance, double speed, int timeout, LinearOpMode opMode){

        double initPos = drive.br.getCurrentPosition();
        ElapsedTime runtime = new ElapsedTime();
        while (opMode.opModeIsActive() && runtime.seconds() < timeout && (drive.br.getCurrentPosition() - initPos) < Math.abs(distance)){
            opMode.telemetry.addData("position", drive.br.getCurrentPosition());
            opMode.telemetry.addData("target", initPos);
            opMode.telemetry.update();
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
        PID pid = new PID(0.005,0.001,0,angle);
        while (opMode.opModeIsActive() && runtime.seconds() < timeout && Math.abs(robot.imu.getAngularOrientation().firstAngle - Math.abs(angle)) > 1){//TODO: maybe change margin of error
            double newPower = pid.loop(robot.imu.getAngularOrientation().firstAngle, runtime.seconds());
            opMode.telemetry.addData("newPower", newPower);
            opMode.telemetry.addData("target", angle);
            opMode.telemetry.update();
            drive.setMotorPowers(newPower,newPower,-newPower,-newPower);
            opMode.telemetry.addData("Motor spin", newPower);
            opMode.telemetry.addData("angle", robot.imu.getAngularOrientation().firstAngle);
        }

        drive.setMotorPowers(0,0,0,0);
    }
}
