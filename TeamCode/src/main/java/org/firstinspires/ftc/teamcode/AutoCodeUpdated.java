package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous/*(preselectTeleOp="DriveCode")*/
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
