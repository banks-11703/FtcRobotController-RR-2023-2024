package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

import java.math.MathContext;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while(opModeIsActive() && !isStopRequested()) {
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(new Vector2d(36, 36),Math.toRadians(0))
//                                .turnTo(Math.toRadians(90))
                                .strafeToLinearHeading(new Vector2d(48, 0),Math.toRadians(0))
                                .turnTo(drive.pose.heading.plus(Math.toRadians(90)))
                                .strafeToLinearHeading(new Vector2d(0,0),Math.toRadians(0))
//                                .turnTo(Math.toRadians(0))
                                .build());
                drive.updatePoseEstimate();
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
