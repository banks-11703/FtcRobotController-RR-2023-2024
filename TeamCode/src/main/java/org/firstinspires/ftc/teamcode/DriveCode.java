package org.firstinspires.ftc.teamcode;

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
            updateValues(drive);
            pidDriving(drive);
            intake(drive);
            outtake(drive);
            lift(drive);
            telemetry(drive);
            launcher(drive);
            lights(drive);
        }
    }
}
