package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous/*(preselectTeleOp="DriveCode")*///TODO uncomment
public class AutoCodeUpdated extends AutoCodeCommonUpdated {

    @Override
    public void runOpMode() throws InterruptedException {
        setup();
        initialization();
        MecanumDrive drive = new MecanumDrive(hardwareMap, finalStart);
        waitForStart();
        liftSetup(drive);
        runAutoPreloaded(drive);
    }
}
