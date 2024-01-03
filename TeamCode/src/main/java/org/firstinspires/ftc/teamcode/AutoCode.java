package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp="DriveCode")
public class AutoCode extends AutoCodeCommon {

    @Override
    public void runOpMode() throws InterruptedException {
        setup();
        initialization();
        initAprilTag();
        MecanumDrive drive = new MecanumDrive(hardwareMap, finalStart);
        setManualExposure(6, 250);
        waitForStart();
        liftSetup(drive);
        runAutoPreloaded(drive);
        drive = new MecanumDrive(hardwareMap, updatedPose);
        park(drive);
    }
}
