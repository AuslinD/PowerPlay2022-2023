package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Mecanum extends OpMode {

    Robot robot;

    @Override
    public void init() {
        robot = new Robot(this);

    }

    @Override
    public void loop() {
        robot.drivetrain.teleOpControls(gamepad1);
    }
}
