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
        MecanumDrive drive = new MecanumDrive(hardwareMap, finalStart);
        waitForStart();
//        buildTrajectories(drive);
        scorePreloadedFloor(drive);
        driveToBackStage(drive);
//        scorePreloadedBackdrop(drive);
        park(drive);
    }
}
