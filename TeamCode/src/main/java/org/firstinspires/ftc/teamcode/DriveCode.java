package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Config
public class DriveCode extends DriveCodeCommon {


    @Override
    public void runOpMode() {
        Initialization();
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            updateButtons();
            rawDriving();
            intake();
            outake();
            lift();
            telemetry();
        }
    }
}