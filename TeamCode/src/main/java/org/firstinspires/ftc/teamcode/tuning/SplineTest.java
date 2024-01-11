package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

import java.math.MathContext;

@TeleOp
@Disabled
public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, 60, Math.toRadians(-90)));

            waitForStart();

            while (opModeIsActive() && !isStopRequested()) {
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(12, 12), Math.toRadians(-90))
                        .build()
                );
                drive.updatePoseEstimate();
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(48, 12), Math.toRadians(0))
                        .build()
                );
                drive.updatePoseEstimate();
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(48, 60), Math.toRadians(0))
                        .build()
                );
                drive.updatePoseEstimate();
//                Actions.runBlocking(drive.actionBuilder(drive.pose)
//                        .turnTo(Math.toRadians(180))
//                        .build()
//                );
//                drive.updatePoseEstimate();
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(12, 60), Math.toRadians(180))
                        .build()
                );
                drive.updatePoseEstimate();
//                Actions.runBlocking(drive.actionBuilder(drive.pose)
//                        .turnTo(Math.toRadians(-90))
//                        .build()
//                );
//                drive.updatePoseEstimate();
            }

        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(60, 0), Math.PI)
                            .build());
        } else {
            throw new AssertionError();
        }
    }
}
