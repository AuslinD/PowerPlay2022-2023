package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class PurePursuitOpMode extends OpMode {
    @Override
    public void init() {


    }

    @Override
    public void loop() {
        PurePursuitMovement.goToPosition(358 / 2, 358 / 2, 0.3);
    }
}
