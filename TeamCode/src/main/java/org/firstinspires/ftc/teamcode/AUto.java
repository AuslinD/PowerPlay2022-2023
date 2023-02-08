package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

 public class AUto{
    Manipulator manipulator;
    private Drivetrain drive;
    Robot robot;
    double initHeading;


    public AUto(Robot a){
        robot = a;
        drive = robot.getDrivetrain();
    }

    public void drive(double distance, int timeout, LinearOpMode opMode){

        double initPos = drive.br.getCurrentPosition();
        ElapsedTime runtime = new ElapsedTime();
        PID pid = new PID(0.08,0.0008,0.001,distance);
        while (opMode.opModeIsActive() && runtime.seconds() < timeout && Math.abs(drive.br.getCurrentPosition() - initPos) < Math.abs(distance)){
            opMode.telemetry.addData("position br ", drive.br.getCurrentPosition());
            opMode.telemetry.addData("position bl ", drive.bl.getCurrentPosition());
            opMode.telemetry.addData("position fr ", drive.fr.getCurrentPosition());
            opMode.telemetry.addData("position fl ", drive.fl.getCurrentPosition());
            opMode.telemetry.addData("target", initPos);
            opMode.telemetry.addData("distance til: ", drive.br.getCurrentPosition() - initPos);
            double newPower = pid.loop(drive.br.getCurrentPosition() - initPos, runtime.seconds());
            newPower = newPower * 0.475;
            drive.setMotorPowers(-newPower,newPower, newPower,-newPower);
            opMode.telemetry.addData("newpower ",newPower);
            opMode.telemetry.update();
        }
        drive.setAllMotors(0);
    }

     public void PIDDrive(double distance, double p, double i, double d, int timeout, LinearOpMode opMode){

         double initPos = drive.br.getCurrentPosition();
         ElapsedTime runtime = new ElapsedTime();
         PID pid = new PID(p, i, d,distance);
         while (opMode.opModeIsActive() && runtime.seconds() < timeout && Math.abs(drive.br.getCurrentPosition() - initPos) < Math.abs(distance)){
             opMode.telemetry.addData("position br ", drive.br.getCurrentPosition());
             opMode.telemetry.addData("position bl ", drive.bl.getCurrentPosition());
             opMode.telemetry.addData("position fr ", drive.fr.getCurrentPosition());
             opMode.telemetry.addData("position fl ", drive.fl.getCurrentPosition());
             opMode.telemetry.addData("target", initPos);
             opMode.telemetry.addData("distance til: ", drive.br.getCurrentPosition() - initPos);
             double newPower = pid.loop(drive.br.getCurrentPosition() - initPos, runtime.seconds());
             newPower = newPower * 0.5;
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
        PID pid = new PID(0.007,0.003,0.001, angle);
        while (opMode.opModeIsActive() && runtime.seconds() < timeout && Math.abs(robot.imu.getAngularOrientation().firstAngle - Math.abs(angle)) > 1.5){//TODO: maybe change margin of error
            double newPower = pid.loop(robot.imu.getAngularOrientation().firstAngle, runtime.seconds()) * .8;
            opMode.telemetry.addData("newPower", newPower);
            opMode.telemetry.addData("target", angle);
            opMode.telemetry.update();
            drive.setMotorPowers(newPower,newPower,-newPower,-newPower);
            opMode.telemetry.addData("Motor spin", newPower);
            opMode.telemetry.addData("angle", robot.imu.getAngularOrientation().firstAngle);
        }

        drive.setMotorPowers(0,0,0,0);
    }

    public void toHeading(double angle, int timeout, LinearOpMode opMode){
        ElapsedTime runtime = new ElapsedTime();
        opMode.telemetry.addData("loop",  Math.abs(robot.imu.getAngularOrientation().firstAngle - angle));
        opMode.telemetry.update();
        PID pid = new PID(0.015,0.008,0.003, initHeading + angle);
        while(opMode.opModeIsActive() && runtime.seconds() < timeout && Math.abs(robot.imu.getAngularOrientation().firstAngle - angle) > 0.25){
            double newPower = pid.loop(robot.imu.getAngularOrientation().firstAngle, runtime.seconds()) * .8;

            opMode.telemetry.addData("initHeading", initHeading);
            opMode.telemetry.addData("newPower", newPower);
            opMode.telemetry.addData("target", initHeading + angle);
            opMode.telemetry.update();
            newPower *= .9;
            drive.setMotorPowers(newPower,newPower,-newPower,-newPower);
            opMode.telemetry.addData("Motor power", newPower);
            opMode.telemetry.addData("angle", robot.imu.getAngularOrientation().firstAngle);
            opMode.telemetry.addData("condition", Math.abs(robot.imu.getAngularOrientation().firstAngle - angle));
        }
    }
    public void driveOdom(double distance, int timeout, LinearOpMode opMode){
        double initPos = drive.getWheelPositions().get(0);
        ElapsedTime runtime = new ElapsedTime();
        PID pid = new PID(0.13,0.005,0.001,distance);
        while (opMode.opModeIsActive() && runtime.seconds() < timeout && Math.abs((drive.getWheelPositions().get(0)) - initPos) < Math.abs(distance)){
            opMode.telemetry.addData("position 0 ", drive.getWheelPositions().get(0));
            opMode.telemetry.addData("position 0 ", drive.getWheelPositions().get(0));
            //opMode.telemetry.addData("position 1 ", drive.getWheelPositions().get(1));
            //opMode.telemetry.addData("position 1 ", drive.getWheelPositions().get(1));
            opMode.telemetry.addData("target", initPos);
            opMode.telemetry.addData("distance til: ", Math.abs((drive.getWheelPositions().get(0)) - initPos) - Math.abs(distance));
            double newPower = pid.loop(drive.getWheelPositions().get(0) - initPos, runtime.seconds());
            newPower = newPower * 0.475;
            drive.setMotorPowers(-newPower,newPower, newPower,-newPower);
            opMode.telemetry.addData("newpower ",newPower);
            opMode.telemetry.update();
        }
        drive.setAllMotors(0);
    }
}
