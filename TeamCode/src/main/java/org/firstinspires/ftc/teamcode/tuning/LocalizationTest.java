package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

@TeleOp
//@Disabled
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            telemetry = FtcDashboard.getInstance().getTelemetry();
//            drive.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            drive.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            drive.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            drive.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            drive.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            drive.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            drive.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            drive.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", drive.pose.heading);
//                telemetry.addData("par0",drive.frontRight.getCurrentPosition());
//                telemetry.addData("par1",drive.frontLeft.getCurrentPosition());
//                telemetry.addData("perp", drive.backRight.getCurrentPosition());
                telemetry.update();



            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                0.0
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", drive.pose.heading);
                telemetry.update();
            }
        } else {
            throw new AssertionError();
        }
    }
}
