package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
//@Config
public class DriveCode extends DriveCodeCommon {


    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Initialization(drive);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            updateButtons(drive);
            rawDriving(drive);
            intake(drive);
            outake(drive);
            lift(drive);
            telemetry(drive);
            launcher(drive);
        }
    }
}
