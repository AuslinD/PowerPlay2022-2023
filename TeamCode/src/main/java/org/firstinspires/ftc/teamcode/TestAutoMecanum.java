package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Mecanum auto test", group = "Auto")
public class TestAutoMecanum extends LinearOpMode{
    Drivetrain yes = new Drivetrain(this);
    AUto no = new AUto(yes);
    no.drive()
    @Override
    public void runOpMode() throws InterruptedException {

    }
}
