package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MecanumDrive.PARAMS;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;
import java.util.List;

@Autonomous
@Disabled
public class AutoCodeCommon extends LinearOpMode {

    OpenCvCamera camera;
    public ColorDetection.RedDeterminationPipeline pipelineRed;
    public ColorDetection.BlueDeterminationPipeline pipelineBlue;

    private AprilTagProcessor aprilTag;
    VisionPortal visionPortal;

    public static double aprilTagXMod = 1;
    public static double aprilTagYMod = 1;
    public static double aprilTagHeadingMod = 1;

    boolean lockedIn = false;
    boolean wasLockedIn = false;

    int team = 0;

    int Team() {
        return team % 2;
    }//0==Blue   1==Red

    int side = 0;

    int Side() {
        return side % 2;
    }  //0==BackStage   1==FrontStage

    GamepadEx a1 = new GamepadEx();
    GamepadEx b1 = new GamepadEx();
    GamepadEx x1 = new GamepadEx();
    GamepadEx y1 = new GamepadEx();

    int randomizationResult;//0==left   1==center   2==right


    int yMod = 1;
    int xMod = 0;
    int headingMod = 0;

    Pose2d startRedLeft = new Pose2d(-36, -63.9375, Math.toRadians(-90));
    Pose2d startRedRight = new Pose2d(36, -63.9375, Math.toRadians(-90));
    Pose2d startBlueLeft = new Pose2d(36, 63.9375, Math.toRadians(90));
    Pose2d startBlueRight = new Pose2d(-36, 63.9375, Math.toRadians(90));
    Pose2d finalStart;

    Vector2d parkLower;
    Vector2d parkUpper;
    Vector2d scoreClose;
    Vector2d scoreMiddle;
    Vector2d scoreFar;
    Vector2d spikeLeft;
    Vector2d spikeCenter;
    Vector2d spikeRight;

    int targetId = 1;


    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void setup() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipelineRed = new ColorDetection.RedDeterminationPipeline();
        pipelineBlue = new ColorDetection.BlueDeterminationPipeline();
        camera.setPipeline(pipelineBlue);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public void initialization() {
        while (!opModeIsActive() && !isStopRequested()) {
            a1.updateButton(gamepad1.a);
            b1.updateButton(gamepad1.b);
            x1.updateButton(gamepad1.x);
            y1.updateButton(gamepad1.y);

            if (lockedIn) {
                telemetry.addData("Press A to back out", "");
            } else {
                telemetry.addData("Press A to lock in decision\n" +
                        "Press B to change team\n" +
                        "Press X to change side", "");
            }
            if (Team() == 0) {
                telemetry.addData("Team", "Blue");
            } else {
                telemetry.addData("Team", "Red");
            }
            if (Side() == 0) {
                telemetry.addData("Side", "BackStage");
            } else {
                telemetry.addData("Side", "FrontStage");
            }

            if (lockedIn) {
                if (Team() == 0) {
                    camera.setPipeline(pipelineBlue);
                    telemetry.addData("Analysis", pipelineBlue.getAnalysis());
                } else {
                    camera.setPipeline(pipelineRed);
                    telemetry.addData("Analysis", pipelineRed.getAnalysis());
                }
                if (!wasLockedIn) {
                    positionAnalysis();
                    lockedIn = true;
                }
            } else {
                if (b1.isPressed()) {
                    team++;
                }

                if (x1.isPressed()) {
                    side++;
                }

                wasLockedIn = false;
            }

            if (a1.isPressed()) {
                lockedIn = !lockedIn;
            }

            telemetry.update();
        }
        if (Team() == 0) {
            if (pipelineBlue.getAnalysis() == ColorDetection.BlueDeterminationPipeline.TeamElementPosition.LEFT) {
                randomizationResult = 0;
            } else if (pipelineBlue.getAnalysis() == ColorDetection.BlueDeterminationPipeline.TeamElementPosition.CENTER) {
                randomizationResult = 1;
            } else if (pipelineBlue.getAnalysis() == ColorDetection.BlueDeterminationPipeline.TeamElementPosition.RIGHT) {
                randomizationResult = 2;
            }
        } else {
            if (pipelineRed.getAnalysis() == ColorDetection.RedDeterminationPipeline.TeamElementPosition.LEFT) {
                randomizationResult = 0;
            } else if (pipelineRed.getAnalysis() == ColorDetection.RedDeterminationPipeline.TeamElementPosition.CENTER) {
                randomizationResult = 1;
            } else if (pipelineRed.getAnalysis() == ColorDetection.RedDeterminationPipeline.TeamElementPosition.RIGHT) {
                randomizationResult = 2;
            }
        }
    }

    public void positionAnalysis() {
        if (Team() == 0 && Side() == 0) {//blue backStage
            yMod = -1;
            xMod = 72;
            headingMod = 180;
            finalStart = startBlueLeft;
        } else if (Team() == 0 && Side() == 1) {//blue frontStage
            yMod = -1;
            xMod = 0;
            headingMod = 180;
            finalStart = startBlueRight;
        } else if (Team() == 1 && Side() == 0) {//red frontStage
            yMod = 1;
            xMod = 0;
            headingMod = 0;
            finalStart = startRedLeft;
        } else if (Team() == 1 && Side() == 1) {//red backStage
            yMod = 1;
            xMod = 72;
            headingMod = 0;
            finalStart = startRedRight;
        }

        //all positions with y,x,or heading Mod assume red left starting
        parkLower = new Vector2d(60, -63 * yMod);
        parkUpper = new Vector2d(60, -12 * yMod);
        scoreClose = new Vector2d(-36 + xMod, -60 * yMod);
        scoreMiddle = new Vector2d(-36 + xMod, -60 * yMod);
        scoreFar = new Vector2d(-36 + xMod, -60 * yMod);
        spikeLeft = new Vector2d(-36 + xMod, -33 * yMod);
        spikeCenter = new Vector2d(-36 + xMod, -12 * yMod);
        spikeRight = new Vector2d(-36 + xMod, -33 * yMod);
    }

    public void buildTrajectories(MecanumDrive drive) {

    }

    public void scorePreloadedFloor(MecanumDrive drive) {
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(finalStart.position.x, finalStart.position.y + 5 * yMod), finalStart.heading)
                .build()
        );
        drive.updatePoseEstimate();
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .turnTo(-90 * randomizationResult + 180 + headingMod)
                .build()
        );
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(spikeRight, Math.toRadians(-90 * randomizationResult + 180 + headingMod))
                .build());
        drive.updatePoseEstimate();
        if (randomizationResult != 1) {
            drive.intake.setPower(0.5);
            sleep(1000);
            drive.intake.setPower(0);
        } else {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(spikeCenter, Math.toRadians(-90 * randomizationResult + 180 + headingMod))
                    .build()
            );
            drive.intake.setPower(0.5);
            sleep(1000);
            drive.intake.setPower(0);
        }

//        switch (randomizationResult) {
//            case 0:
//                Actions.runBlocking(drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(spikeCenter, Math.toRadians(-90 * yMod))
//                        .turn(Math.toRadians(90))
//                        .build());
//                drive.updatePoseEstimate();
//                Actions.runBlocking(drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(spikeLeft, Math.toRadians(headingMod))
//                        .build());
//                drive.updatePoseEstimate();
//                drive.intake.setPower(0.5);
//                Actions.runBlocking(new SleepAction(1));
////                drive.intake.setPower(0.3);
////                Actions.runBlocking(new SleepAction(1));
//                Actions.runBlocking(drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(spikeCenter, Math.toRadians(headingMod))
//                        .build());
//                drive.updatePoseEstimate();
//                Actions.runBlocking(drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(new Vector2d(-36 + xMod, -50 * yMod), Math.toRadians(headingMod))
//                        .build());
//                drive.updatePoseEstimate();
//                Actions.runBlocking(drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(new Vector2d(-42 + xMod, -63 * yMod), Math.toRadians(0))
//                        .build());
//                drive.updatePoseEstimate();
//                break;
//            case 1:
//                Actions.runBlocking(drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(spikeCenter, Math.toRadians(-90 * yMod))
//                        .build());
//                drive.updatePoseEstimate();
//                drive.intake.setPower(0.3);
//                Actions.runBlocking(new SleepAction(1));
//                Actions.runBlocking(drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(new Vector2d(-42 + xMod, -63 * yMod), Math.toRadians(0))
//                        .turnTo(Math.toRadians(0))
//                        .build());
//                drive.updatePoseEstimate();
//                break;
//            case 2:
//                Actions.runBlocking(drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(spikeCenter, Math.toRadians(-90 * yMod))
//                        .turn(Math.toRadians(-90))
//                        .build());
//                drive.updatePoseEstimate();
//                Actions.runBlocking(drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(spikeRight, Math.toRadians(180 + headingMod))
//                        .build());
//                drive.updatePoseEstimate();
//                drive.intake.setPower(0.3);
//                Actions.runBlocking(new SleepAction(1));
//                if (Team() == 1 || Side() == 0) {
//                    Actions.runBlocking(drive.actionBuilder(drive.pose)
//                            .strafeToLinearHeading(spikeCenter, Math.toRadians(180 + headingMod))
//                            .build());
//                    drive.updatePoseEstimate();
//                    Actions.runBlocking(drive.actionBuilder(drive.pose)
//                            .strafeToLinearHeading(new Vector2d(-42 + xMod, -63 * yMod), Math.toRadians(0))
//                            .turnTo(Math.toRadians(0))
//                            .build());
//                    drive.updatePoseEstimate();
//                } else {
//                    Actions.runBlocking(drive.actionBuilder(drive.pose)
//                            .strafeToLinearHeading(spikeCenter, Math.toRadians(0))
//                            .build());
//                    drive.updatePoseEstimate();
//                    Actions.runBlocking(drive.actionBuilder(drive.pose)
//                            .strafeToLinearHeading(new Vector2d(-42 + xMod, -63 * yMod), Math.toRadians(180))
//                            .turnTo(Math.toRadians(0))
//                            .build());
//                    drive.updatePoseEstimate();
//                }
//                break;
//        }

    }

    public void driveToBackStage(MecanumDrive drive) {
//        if ((Team() == 0 && Side() == 1) || (Team() == 1 && Side() == 0)) {
//            Actions.runBlocking(drive.actionBuilder(drive.pose)
//                    .strafeToLinearHeading(new Vector2d(36, -63 * yMod), drive.pose.heading)
//                    .build());
//            drive.updatePoseEstimate();
//        }
    }

    public void scorePreloadedBackdrop(MecanumDrive drive) {

    }

    public void park(MecanumDrive drive) {

    }

    public void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 2"), aprilTag);

    }

    public void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }

    public void goToAprilTag(MecanumDrive drive) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        boolean done = false;
        while (!done && !isStopRequested() && opModeIsActive()) {
            double xPower = 0;
            double yPower = 0;
            double rotationPower = 0;
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == targetId) {
                    xPower = detection.ftcPose.x / aprilTagXMod;
                    yPower = (detection.ftcPose.y - 10) / aprilTagYMod;
                    rotationPower = detection.ftcPose.yaw / aprilTagHeadingMod;
                    if (Math.abs(detection.ftcPose.x) <= 1 &&
                        Math.abs(detection.ftcPose.y - 10) <= 1 &&
                        Math.abs(detection.ftcPose.yaw - 90) <= 5) {
                        done = true;
                    }
                }
            }
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            yPower,
                            xPower
                    ),
                    rotationPower
            ));

            drive.updatePoseEstimate();


        }

    }

}
