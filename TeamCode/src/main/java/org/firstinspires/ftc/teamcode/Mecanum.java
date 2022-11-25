package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TeleOp", group = "actual")
public class Mecanum extends OpMode {

    Robot robot;
    private DistanceSensor distance;

    @Override
    public void init() {
        robot = new Robot(this);
        distance = hardwareMap.get(DistanceSensor.class, "sensor_range");
    }

    @Override
    public void loop() {
        robot.drivetrain.teleOpControls(gamepad1);
        robot.manip.teleOpControls(gamepad2);
        /*robot.manip.distanceSensor(gamepad2, distance.getDistance(DistanceUnit.CM));
        telemetry.addData("distance", distance.getDistance(DistanceUnit.CM));
        telemetry.update();*/
    }
}
