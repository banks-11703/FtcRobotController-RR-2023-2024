package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
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
        scorePreloadedFloor(drive);
        driveToBackStage(drive);
        goToAprilTag(drive);
        drive = new MecanumDrive(hardwareMap,updatedPose);
        scorePreloadedBackdrop(drive);
        park(drive);
    }
}
