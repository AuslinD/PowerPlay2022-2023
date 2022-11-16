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

    public void drive(double distance, int timeout, LinearOpMode opMode){

        double initPos = drive.br.getCurrentPosition();
        ElapsedTime runtime = new ElapsedTime();
        PID pid = new PID(0.03,0.0006,0.0004,distance);
        while (opMode.opModeIsActive() && runtime.seconds() < timeout && Math.abs(drive.br.getCurrentPosition() - initPos) < Math.abs(distance)){
            opMode.telemetry.addData("position br ", drive.br.getCurrentPosition());
            opMode.telemetry.addData("position bl ", drive.bl.getCurrentPosition());
            opMode.telemetry.addData("position fr ", drive.fr.getCurrentPosition());
            opMode.telemetry.addData("position fl ", drive.fl.getCurrentPosition());
            opMode.telemetry.addData("target", initPos);
            opMode.telemetry.addData("distance til: ", drive.br.getCurrentPosition() - initPos);
            double newPower = pid.loop(drive.br.getCurrentPosition() - initPos, runtime.seconds());

            drive.setMotorPowers(-newPower,newPower, newPower,-newPower);
            opMode.telemetry.addData("newpower ",newPower);
            opMode.telemetry.update();
        }
        drive.setAllMotors(0);
    }

    public void turn(double angle, int timeout, LinearOpMode opMode){
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
