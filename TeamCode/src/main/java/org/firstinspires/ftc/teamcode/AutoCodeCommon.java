package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous
@Disabled
public class AutoCodeCommon extends LinearOpMode {

    OpenCvCamera camera;
    public ColorDetection.RedDeterminationPipeline pipelineRed;
    public ColorDetection.BlueDeterminationPipeline pipelineBlue;

    // Adjust these numbers to suit your robot.
    static final double DESIRED_DISTANCE = 9; //  this is how close the camera should get to the target (inches)
    static final double DESIRED_BEARING = 0;
    static final double DESIRED_YAW = -4;

    static final double DISTANCE_F = 0.075;
    static final double BEARING_F = 0.2;
    static final double YAW_F = 0.10;

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    static final double SPEED_GAIN = 0.04;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    static final double STRAFE_GAIN = 0.04;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    static final double TURN_GAIN = 0.02;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.3;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.3;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.2;   //  Clip the turn speed to this max value (adjust for your robot)

    boolean backupTagBehind = false;
    boolean backupTag2Behind = true;
    boolean tagEverFound = false;
    int BACKUP_TAG2_ID = -1;
    int BACKUP_TAG_ID = -1;
    int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


    private double desiredTagRange = 0;
    private double desiredTagBearing = 0;
    private double desiredTagYaw = 0;

    ElapsedTime timeSinceAprilTag = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime timeSinceSeen = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime timeSinceBackupTag = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime timeSinceBackupTag2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    boolean lockedIn = false;
    boolean wasLockedIn = false;

    int team = 0;

    /**
     * 0==Blue
     * 1==Red
     */
    int Team() {
        return team % 2;
    }

    int side = 0;

    /**
     * 0==FrontStage
     * 1==BackStage
     */
    int Side() {
        return side % 2;
    }

    int park = 0;

    /**
     * 0==Far
     * 1==Close
     */
    int Park() {
        return park % 2;
    }

    GamepadEx a1 = new GamepadEx();
    GamepadEx b1 = new GamepadEx();
    GamepadEx x1 = new GamepadEx();
    GamepadEx y1 = new GamepadEx();

    int randomizationResult;//0==left   1==center   2==right


    int yMod = 1;
    int xMod = 0;
    int headingMod = 0;

    double flipperScore = 0.32;
    double flipperIntake = 0.86;
    double flipperHold = 0.6;

    final Pose2d startRedLeft = new Pose2d(-36, -63.9375, Math.toRadians(-90));
    final Pose2d startRedRight = new Pose2d(12, -63.9375, Math.toRadians(-90));
    final Pose2d startBlueLeft = new Pose2d(12, 63.9375, Math.toRadians(90));
    final Pose2d startBlueRight = new Pose2d(-36, 63.9375, Math.toRadians(90));
    Pose2d finalStart = new Pose2d(0, 0, 0);
    Pose2d updatedPose = new Pose2d(0, 0, 0);
    final Vector2d backdropPos1 = new Vector2d(42, 42);
    final Vector2d backdropPos2 = new Vector2d(42, 36);
    final Vector2d backdropPos3 = new Vector2d(42, 30);
    final Vector2d backdropPos4 = new Vector2d(42, -26);
    final Vector2d backdropPos5 = new Vector2d(42, -26);
    final Vector2d backdropPos6 = new Vector2d(42, -26);
    Vector2d finalBackdropPos = new Vector2d(0, 0);

    Vector2d parkLower;
    Vector2d parkUpper;
    Vector2d scoreClose;
    Vector2d scoreMiddle;
    Vector2d scoreFar;
    Vector2d spikePosUpperFront;
    Vector2d spikePosLowerFront;
    Vector2d spikePosUpperBack;
    Vector2d spikePosLowerBack;


    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void setup() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipelineRed = new ColorDetection.RedDeterminationPipeline();
        pipelineBlue = new ColorDetection.BlueDeterminationPipeline();
        camera.setPipeline(pipelineBlue);
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
                        "Press X to change side\n" +
                        "Press Y to change parking location", "");
            }
            if (Team() == 0) {
                telemetry.addData("Team", "Blue");
            } else {
                telemetry.addData("Team", "Red");
            }
            if (Side() == 0) {
                telemetry.addData("Side", "FrontStage");
            } else {
                telemetry.addData("Side", "BackStage");
            }

            if (Park() == 0) {
                telemetry.addData("Park", "Far");
            } else {
                telemetry.addData("Park", "Close");
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
                if (y1.isPressed()) {
                    park++;
                }

                wasLockedIn = false;
            }

            if (a1.isPressed()) {
                telemetry.clearAll();
                lockedIn = !lockedIn;
            }

            telemetry.update();
            if (getRuntime() % 10 == 0) {
                telemetry.clearAll();
            }
        }

        if (Team() == 0) {
            if (pipelineBlue.getAnalysis() == ColorDetection.BlueDeterminationPipeline.TeamElementPosition.LEFT) {
                randomizationResult = 0;
                DESIRED_TAG_ID = 1;
                BACKUP_TAG_ID = 2;
                backupTagBehind = true;
                BACKUP_TAG2_ID = 3;
                backupTag2Behind = true;
                finalBackdropPos = backdropPos1;
            } else if (pipelineBlue.getAnalysis() == ColorDetection.BlueDeterminationPipeline.TeamElementPosition.CENTER) {
                randomizationResult = 1;
                DESIRED_TAG_ID = 2;
                BACKUP_TAG_ID = 1;
                backupTagBehind = false;
                BACKUP_TAG2_ID = 3;
                backupTag2Behind = true;

                finalBackdropPos = backdropPos2;
            } else if (pipelineBlue.getAnalysis() == ColorDetection.BlueDeterminationPipeline.TeamElementPosition.RIGHT) {
                randomizationResult = 2;
                DESIRED_TAG_ID = 3;
                BACKUP_TAG_ID = 2;
                backupTagBehind = false;
                BACKUP_TAG2_ID = 1;
                backupTag2Behind = false;
                finalBackdropPos = backdropPos3;
            }
        } else {
            if (pipelineRed.getAnalysis() == ColorDetection.RedDeterminationPipeline.TeamElementPosition.LEFT) {
                randomizationResult = 0;
                DESIRED_TAG_ID = 4;
                BACKUP_TAG_ID = 5;
                backupTagBehind = false;
                BACKUP_TAG2_ID = 6;
                backupTag2Behind = false;
                finalBackdropPos = backdropPos4;
            } else if (pipelineRed.getAnalysis() == ColorDetection.RedDeterminationPipeline.TeamElementPosition.CENTER) {
                randomizationResult = 1;
                DESIRED_TAG_ID = 5;
                BACKUP_TAG_ID = 6;
                backupTagBehind = false;
                BACKUP_TAG2_ID = 4;
                backupTag2Behind = true;
                finalBackdropPos = backdropPos5;
            } else if (pipelineRed.getAnalysis() == ColorDetection.RedDeterminationPipeline.TeamElementPosition.RIGHT) {
                randomizationResult = 2;
                DESIRED_TAG_ID = 6;
                BACKUP_TAG_ID = 5;
                backupTagBehind = true;
                BACKUP_TAG2_ID = 4;
                backupTag2Behind = true;
                finalBackdropPos = backdropPos6;
            }
        }
        camera.closeCameraDevice();
    }

    public void positionAnalysis() {
        if (Team() == 0 && Side() == 0) {//blue frontStage
            yMod = -1;
            xMod = 0;
            headingMod = 180;
            finalStart = startBlueRight;
        } else if (Team() == 0 && Side() == 1) {//blue backstage
            yMod = -1;
            xMod = 72;
            headingMod = 180;
            finalStart = startBlueLeft;
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
        spikePosUpperFront = new Vector2d(-36 + xMod, -26.5 * yMod);
        spikePosLowerFront = new Vector2d(-36 + xMod, -35 * yMod);
        spikePosUpperBack = new Vector2d(12, -26.5 * yMod);
        spikePosLowerBack = new Vector2d(12, -35 * yMod);
    }

    public void liftSetup(@NonNull MecanumDrive drive) {
        drive.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftLift.setTargetPosition(0);
        drive.rightLift.setTargetPosition(0);
        drive.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runAutoPreloaded(MecanumDrive drive) {
        if (finalStart.position.x == 0) {
            waitEx(1000000);
        }

        if (Team() == 0 && Side() == 0) {//Blue Frontstage
            autoPreloadedBlueFrontStage(drive);
        } else if (Team() == 0 && Side() == 1) {//Blue Backstage
            autoPreloadedBlueBackStage(drive);
        } else if (Team() == 1 && Side() == 0) {//Red Frontstage
            autoPreloadedRedFrontStage(drive);
        } else if (Team() == 1 && Side() == 1) {//Red Backstage
            autoPreloadedRedBackStage(drive);
        }

        goToAprilTag(drive);
        if (tagEverFound) {
            scoreBackdrop(drive);
        }
    }

    private void autoPreloadedBlueFrontStage(@NonNull MecanumDrive drive) {
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(finalStart.position.x, finalStart.position.y - 5), finalStart.heading)
                .strafeToLinearHeading(new Vector2d(finalStart.position.x, finalStart.position.y - 6), Math.toRadians(-90 * randomizationResult))
                .build()
        );
        lifts(100, drive);
        switch (randomizationResult) {
            case 0://left
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosLowerFront, Math.toRadians(90 * randomizationResult))
                        .strafeToLinearHeading(spikePosLowerFront.plus(new Vector2d(8 - 8 * randomizationResult, 0)), Math.toRadians(90 * randomizationResult))
                        .build());
                drive.updatePoseEstimate();
                drive.flipper.setPosition(0.7);
                waitEx(10);
                lifts(150, drive);
                waitEx(1000);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosLowerFront, Math.toRadians(90 * randomizationResult))
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x, 12), Math.toRadians(90))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
            case 1://center
            default:
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosUpperFront, Math.toRadians(-90 * randomizationResult))
                        .build()
                );
                drive.updatePoseEstimate();
                drive.flipper.setPosition(0.7);
                waitEx(10);
                lifts(150, drive);
                waitEx(1000);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosLowerFront.plus(new Vector2d(0, 8)), Math.toRadians(-90 * randomizationResult))
                        .strafeToLinearHeading(new Vector2d(-60, spikePosLowerBack.y), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(-60, 12), Math.toRadians(90))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
            case 2://right
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosLowerFront, Math.toRadians(90 * randomizationResult))
                        .strafeToLinearHeading(spikePosLowerFront.plus(new Vector2d(8 - 8 * randomizationResult, 0)), Math.toRadians(90 * randomizationResult))
                        .build());
                drive.updatePoseEstimate();
                drive.flipper.setPosition(0.7);
                waitEx(10);
                lifts(150, drive);
                waitEx(1000);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosLowerFront, Math.toRadians(90 * randomizationResult))
                        .strafeToLinearHeading(new Vector2d(spikePosLowerFront.x, 12), Math.toRadians(90))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
        }
        drive.flipper.setPosition(flipperIntake);
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(drive.pose.position.x + 1, 12), Math.toRadians(0))
                .build()
        );
        drive.updatePoseEstimate();
        switch (randomizationResult) {
            case 0://left
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x + 2, 12), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(44, 12 + 2), Math.toRadians(0))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
            case 1://center
            default:
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x + 2, 12), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(44, 12), Math.toRadians(0))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
            case 2://right
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x + 2, 12), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(44, 12 +4), Math.toRadians(0))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
        }
    }

    private void autoPreloadedBlueBackStage(@NonNull MecanumDrive drive) {
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(finalStart.position.x, finalStart.position.y - 5), finalStart.heading)
                .strafeToLinearHeading(new Vector2d(finalStart.position.x, finalStart.position.y - 6), Math.toRadians(-90 * randomizationResult))
                .build()
        );
        lifts(100, drive);
        switch (randomizationResult) {
            case 0://left
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosLowerBack, Math.toRadians(90 * randomizationResult))
                        .strafeToLinearHeading(spikePosLowerBack.plus(new Vector2d(8 - 8 * randomizationResult, 0)), Math.toRadians(90 * randomizationResult))
                        .build());
                drive.updatePoseEstimate();
                drive.flipper.setPosition(0.7);
                waitEx(10);
                lifts(150, drive);
                waitEx(1000);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosLowerBack, Math.toRadians(90 * randomizationResult))
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x, 12), Math.toRadians(90))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
            case 1://center
            default:
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosUpperBack, Math.toRadians(-90 * randomizationResult))
                        .build()
                );
                drive.updatePoseEstimate();
                drive.flipper.setPosition(0.7);
                waitEx(10);
                lifts(150, drive);
                waitEx(1000);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosLowerBack.plus(new Vector2d(0, 8)), Math.toRadians(-90 * randomizationResult))
                        .strafeToLinearHeading(new Vector2d(36, spikePosLowerBack.y), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(36, 12), Math.toRadians(90))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
            case 2://right
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosLowerBack, Math.toRadians(90 * randomizationResult))
                        .strafeToLinearHeading(spikePosLowerBack.plus(new Vector2d(8 - 8 * randomizationResult, 0)), Math.toRadians(90 * randomizationResult))
                        .build());
                drive.updatePoseEstimate();
                drive.flipper.setPosition(0.7);
                waitEx(10);
                lifts(150, drive);
                waitEx(1000);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosLowerBack, Math.toRadians(90 * randomizationResult))
                        .strafeToLinearHeading(new Vector2d(spikePosLowerBack.x, 12), Math.toRadians(90))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
        }
        drive.flipper.setPosition(flipperIntake);
        lifts(0,drive);
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(drive.pose.position.x + 1, 12), Math.toRadians(0))
                .build()
        );
        drive.updatePoseEstimate();
        switch (randomizationResult) {
            case 0://left
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x + 2, 12), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(44, 12 + 2), Math.toRadians(0))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
            case 1://center
            default:
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x + 2, 12), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(44, 12), Math.toRadians(0))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
            case 2://right
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x + 2, 12), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(44, 12 + 4), Math.toRadians(0))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
        }
    }

    private void autoPreloadedRedFrontStage(@NonNull MecanumDrive drive) {
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(finalStart.position.x, finalStart.position.y + 5), finalStart.heading)
                .strafeToLinearHeading(new Vector2d(finalStart.position.x, finalStart.position.y + 6), Math.toRadians(-90 * (randomizationResult) + 180))
                .build()
        );
        drive.updatePoseEstimate();
        lifts(100,drive);
        switch (randomizationResult) {
            case 0://left
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosLowerFront, Math.toRadians(90 * (randomizationResult) - 180))
                        .strafeToLinearHeading(spikePosLowerFront.plus(new Vector2d(-8 + 8 * randomizationResult, 0)), Math.toRadians(90 * (randomizationResult) - 180))
                        .build());
                drive.updatePoseEstimate();
                drive.flipper.setPosition(0.7);
                waitEx(10);
                lifts(150, drive);
                waitEx(1000);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosLowerFront, Math.toRadians(90 * (randomizationResult) - 180))
                        .strafeToLinearHeading(new Vector2d(spikePosLowerFront.x, -12), Math.toRadians(-90))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
            case 1://center
            default:
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosUpperFront, Math.toRadians(-90 * (randomizationResult) + 180))
                        .build()
                );
                drive.updatePoseEstimate();
                drive.flipper.setPosition(0.7);
                waitEx(10);
                lifts(150, drive);
                waitEx(1000);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosLowerFront.plus(new Vector2d(0,-8)), Math.toRadians(-90 * (randomizationResult) + 180))
                                .strafeToLinearHeading(new Vector2d(-60, spikePosLowerFront.y), Math.toRadians(0))
                                .strafeToLinearHeading(new Vector2d(-60,-12),Math.toRadians(-90))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
            case 2://right
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosLowerFront, Math.toRadians(90 * (randomizationResult) - 180))
                        .strafeToLinearHeading(spikePosLowerFront.plus(new Vector2d(-8 + 8 * randomizationResult, 0)), Math.toRadians(90 * (randomizationResult) - 180))
                        .build());
                drive.updatePoseEstimate();
                drive.flipper.setPosition(0.7);
                waitEx(10);
                lifts(150, drive);
                waitEx(1000);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosLowerFront, Math.toRadians(90 * (randomizationResult) - 180))
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x, -12), Math.toRadians(-90))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
        }
        lifts(0,drive);
        drive.flipper.setPosition(flipperIntake);
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(drive.pose.position.x + 1, -12), Math.toRadians(0))
                .build()
        );
        drive.updatePoseEstimate();
        switch (randomizationResult) {
            case 0://left
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x + 2, -12), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(44, -12 -4), Math.toRadians(0))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
            case 1://center
            default:
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x + 2, -12), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(44, -12), Math.toRadians(0))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
            case 2://right
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x + 2, -12), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(25, -12 + 2), Math.toRadians(0))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
        }
    }

    private void autoPreloadedRedBackStage(@NonNull MecanumDrive drive) {
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(finalStart.position.x, finalStart.position.y + 5), finalStart.heading)
                .strafeToLinearHeading(new Vector2d(finalStart.position.x, finalStart.position.y + 6), Math.toRadians(-90 * (randomizationResult) + 180))
                .build()
        );
        drive.updatePoseEstimate();
        lifts(100, drive);
        switch (randomizationResult) {
            case 0://left
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosLowerBack, Math.toRadians(-90 * (randomizationResult) + 180))
                        .strafeToLinearHeading(spikePosLowerBack.plus(new Vector2d(-8 + 8 * randomizationResult, 0)), Math.toRadians(-90 * (randomizationResult) + 180))
                        .build());
                drive.updatePoseEstimate();
                drive.flipper.setPosition(0.7);
                waitEx(10);
                lifts(150, drive);
                waitEx(1000);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosLowerBack, Math.toRadians(90 * (randomizationResult) - 180))
                        .strafeToLinearHeading(new Vector2d(spikePosLowerBack.x, -12), Math.toRadians(-90))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
            case 1://center
            default:
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosUpperBack, Math.toRadians(-90 * (randomizationResult) + 180))
                        .build()
                );
                drive.updatePoseEstimate();
                drive.flipper.setPosition(0.7);
                waitEx(10);
                lifts(150, drive);
                waitEx(1000);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosLowerBack.plus(new Vector2d(0, -8)), Math.toRadians(-90 * randomizationResult + 180))
                        .strafeToLinearHeading(new Vector2d(36, spikePosLowerBack.y), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(36, -12), Math.toRadians(0))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
            case 2://right
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosLowerBack, Math.toRadians(-90 * (randomizationResult) + 180))
                        .strafeToLinearHeading(spikePosLowerBack.plus(new Vector2d(-9 + 9 * randomizationResult, 0)), Math.toRadians(-90 * (randomizationResult) + 180))
                        .build());
                drive.updatePoseEstimate();
                drive.flipper.setPosition(0.7);
                waitEx(10);
                lifts(150, drive);
                waitEx(1000);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(spikePosLowerBack, Math.toRadians(-90 * (randomizationResult) + 180))
                        .strafeToLinearHeading(new Vector2d(spikePosLowerBack.x, -12), Math.toRadians(-90))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
        }
        lifts(0,drive);
        drive.flipper.setPosition(flipperIntake);
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(drive.pose.position.x + 1, -12), Math.toRadians(0))
                .build()
        );
        drive.updatePoseEstimate();
        switch (randomizationResult) {
            case 0://left
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x + 2, -12), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(44, -12 - 4), Math.toRadians(0))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
            case 1://center
            default:
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x + 2, -12), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(38, -12), Math.toRadians(0))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
            case 2://right
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x + 2, -12), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(44, -12 - 2), Math.toRadians(0))
                        .build()
                );
                drive.updatePoseEstimate();
                break;
        }
    }

    public void park(@NonNull MecanumDrive drive) {
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(drive.pose.position.plus(new Vector2d(-5, 0)), Math.toRadians(0))
                .build()
        );
        drive.updatePoseEstimate();
        if (Team() == 0 && Park() == 0) {//Blue Far
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(drive.pose.position.x, 11 + randomizationResult), Math.toRadians(0))
                    .strafeToLinearHeading(new Vector2d(64, 11 + randomizationResult), Math.toRadians(0))
                    .build()
            );
            drive.updatePoseEstimate();
        } else if (Team() == 0 && Park() == 1) {//Blue Close
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(drive.pose.position.x, 63), Math.toRadians(0))
                    .strafeToLinearHeading(new Vector2d(64, 63), Math.toRadians(0))
                    .build()
            );
            drive.updatePoseEstimate();
        } else if (Team() == 1 && Park() == 0) {//Red Far
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(drive.pose.position.x, -11 - randomizationResult), Math.toRadians(0))
                    .strafeToLinearHeading(new Vector2d(64, -11 - randomizationResult), Math.toRadians(0))
                    .build()
            );
            drive.updatePoseEstimate();
        } else if (Team() == 1 && Park() == 1) {//Red Close
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(drive.pose.position.x, -63), Math.toRadians(0))
                    .strafeToLinearHeading(new Vector2d(64, -63), Math.toRadians(0))
                    .build()
            );
            drive.updatePoseEstimate();
        }
        if (!tagEverFound) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToConstantHeading(drive.pose.position.plus(new Vector2d(-5, 0)))
                    .build()
            );
            drive.updatePoseEstimate();
            drive.flipper.setPosition(0.2);
            waitEx(1850);
            drive.flipper.setPosition(flipperIntake);
            waitEx(150);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToConstantHeading(drive.pose.position.plus(new Vector2d(5, 0)))
                    .build()
            );
            drive.updatePoseEstimate();
        }
    }


    private void waitEx(double milliseconds) {
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        time.reset();
        while (time.time(TimeUnit.MILLISECONDS) < milliseconds && opModeIsActive() && !isStopRequested()) {
        }
    }

    private void scoreBackdrop(@NonNull MecanumDrive drive) {
        moveRobot(0.5, 0, 0, drive);
        waitEx(500);
        lifts(200, drive);
        waitEx(500);
        drive.flipper.setPosition(flipperScore);
        waitEx(1850);
        drive.flipper.setPosition(flipperIntake);
        waitEx(150);
        lifts(0, drive);
        waitEx(50);
    }

    private void lifts(int pos, @NonNull MecanumDrive drive) {
        drive.leftLift.setTargetPosition(pos);
        drive.rightLift.setTargetPosition(pos);
        drive.leftLift.setPower(0.5);
        drive.rightLift.setPower(0.5);
    }


    /**
     * Initialize the AprilTag processor.
     */
    public void initAprilTag() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();


        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .addProcessor(aprilTag)
                .build();

    }

    /**
     * Manually set the camera gain and exposure.
     * <p>
     * This can only be called AFTER calling initAprilTag(), and only works for Webcams;
     */
    public void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
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

    private void goToAprilTag(MecanumDrive drive) {
        boolean backupTag2Seen = false;
        boolean backupTagSeen = false;
        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        double forward = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)

        double rangeError = 1000.0;//inches
        double headingError = 1000.0;//degrees
        double yawError = 1000.0;//degrees

        double rangeErrorMargin = 0.5;//inches
        double headingErrorMargin = 1;//degrees
        double yawErrorMargin = 1;//degrees


        // Use low exposure time to reduce motion blur

        String telemetryData = "";

        timeSinceAprilTag.reset();
        desiredTag = null;
        telemetry.addData("Desired Tag", DESIRED_TAG_ID);
        telemetry.update();
        while (timeSinceAprilTag.time(TimeUnit.MILLISECONDS) < 4000 && !(Math.abs(rangeError) <= rangeErrorMargin && Math.abs(headingError) <= headingErrorMargin && Math.abs(yawError) <= yawErrorMargin) && opModeIsActive() && !isStopRequested()) {
//            targetFound = false;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    telemetry.addData("Desired Tag", DESIRED_TAG_ID);
                    telemetry.addData("Tag Seeing", detection.id);
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        timeSinceSeen.reset();
                        targetFound = true;
                        desiredTag = detection;
                        desiredTagRange = desiredTag.ftcPose.range;
                        desiredTagBearing = desiredTag.ftcPose.bearing;
                        desiredTagYaw = desiredTag.ftcPose.yaw;
                        break;  // don't look any further.
                    } else if (detection.id == BACKUP_TAG_ID) {
                        timeSinceBackupTag.reset();
                        backupTagSeen = true;
                        break;
                    } else if (detection.id == BACKUP_TAG2_ID) {
                        timeSinceBackupTag2.reset();
                        backupTag2Seen = true;
                        break;
                    }
                }
            }
            if(targetFound) {
                telemetry.addData("DESIRED TAG SEEN","");
            }
            telemetry.update();

            // If we have found the desired target, Drive to target Automatically .
            if (targetFound || timeSinceSeen.time(TimeUnit.MILLISECONDS) < 1000) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                rangeError = (desiredTagRange - DESIRED_DISTANCE) + DISTANCE_F;
                if (!targetFound) {
                    headingError = 0;
                    yawError = 0;
                } else {
                    headingError = (desiredTagBearing - DESIRED_BEARING) + BEARING_F;
                    yawError = (desiredTagYaw - DESIRED_YAW) + YAW_F;
                }

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                forward = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            } else {
                forward = 0;
                strafe = -0.4 * yMod;
                turn = 0.04 * yMod;
            }


            moveRobot(forward, strafe, turn, drive);
            sleep(10);
            if (!targetFound) {
                if ((!backupTagBehind && backupTagSeen) || (!backupTag2Behind && backupTag2Seen)) {
                    rangeError = 0.0;
                    yawError = 0.0;
                    headingError = 0.0;
                }
            }
        }
        moveRobot(0, 0, 0, drive);
        if (targetFound || backupTagSeen || backupTag2Seen) {
            tagEverFound = true;
        }

        if (!targetFound && (backupTagSeen || backupTag2Seen)) {
            timeSinceAprilTag.reset();
            boolean done = false;
            long duration = 0;
            boolean behind;
            if (backupTagSeen) {
                duration = timeSinceBackupTag.time(TimeUnit.MILLISECONDS);
                behind = backupTagBehind;
            } else {
                duration = timeSinceBackupTag2.time(TimeUnit.MILLISECONDS);
                behind = backupTag2Behind;
            }

            while (timeSinceAprilTag.time(TimeUnit.MILLISECONDS) < duration && !done && opModeIsActive() && !isStopRequested()) {
                moveRobot(0, 0.4 * yMod, -0.04 * yMod, drive);
            }
        }

        if (targetFound) {
            switch (DESIRED_TAG_ID) {
                case 1:
                    updatedPose = new Pose2d(63.34375 - desiredTag.ftcPose.y, 42 - desiredTag.ftcPose.x, Math.toRadians(0 - desiredTag.ftcPose.bearing));
                    break;
                case 2:
                    updatedPose = new Pose2d(63.34375 - desiredTag.ftcPose.y, 36 - desiredTag.ftcPose.x, Math.toRadians(0 - desiredTag.ftcPose.bearing));
                    break;
                case 3:
                    updatedPose = new Pose2d(63.34375 - desiredTag.ftcPose.y, 30 - desiredTag.ftcPose.x, Math.toRadians(0 - desiredTag.ftcPose.bearing));
                    break;
                case 4:
                    updatedPose = new Pose2d(63.34375 - desiredTag.ftcPose.y, -30 - desiredTag.ftcPose.x, Math.toRadians(0 - desiredTag.ftcPose.bearing));
                    break;
                case 5:
                    updatedPose = new Pose2d(63.34375 - desiredTag.ftcPose.y, -36 - desiredTag.ftcPose.x, Math.toRadians(0 - desiredTag.ftcPose.bearing));
                    break;
                case 6:
                    updatedPose = new Pose2d(63.34375 - desiredTag.ftcPose.y, -42 - desiredTag.ftcPose.x, Math.toRadians(0 - desiredTag.ftcPose.bearing));
                    break;
            }
        } else {
            updatedPose = drive.pose;
        }
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    private void moveRobot(double x, double y, double yaw, @NonNull MecanumDrive drive) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }


        // Send powers to the wheels.
        drive.frontLeft.setPower(leftFrontPower);
        drive.frontRight.setPower(rightFrontPower);
        drive.backLeft.setPower(leftBackPower);
        drive.backRight.setPower(rightBackPower);
    }

}
