package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AustinTrollAuto extends LinearOpMode {
    Robot robot;
    AutoMoment automethods;
    @Override
    public void runOpMode() throws InterruptedException {
        automethods = new AutoMoment();
        automethods.drive(0, 0, true, 0);
    }
}
