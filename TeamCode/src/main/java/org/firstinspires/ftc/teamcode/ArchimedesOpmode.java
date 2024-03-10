package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Disabled
//@Config
public class ArchimedesOpmode extends LinearOpMode {


    @Override
    public void runOpMode() {
        final DcMotorEx screw  = hardwareMap.get(DcMotorEx.class, "screw");;
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            if(gamepad1.a) {
                screw.setPower(1);
            } else if(gamepad1.b) {
                screw.setPower(-1);
            } else {
                screw.setPower(0);
            }
        }
    }
}