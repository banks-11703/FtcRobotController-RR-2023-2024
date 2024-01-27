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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.TimeUnit;

@Autonomous
@Disabled
public class AutoCodeCommonUpdated extends LinearOpMode {

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

    int path = 0;

    /**
     * 0==Rigging
     * 1==Stage Door
     */
    int Path() {
        return path % 2;
    }

    boolean finishEarly = false;

    double startDelay = 0.0;
    static final double MAX_DELAY = 30;

    GamepadEx a1 = new GamepadEx();
    GamepadEx b1 = new GamepadEx();
    GamepadEx x1 = new GamepadEx();
    GamepadEx y1 = new GamepadEx();
    GamepadEx l1 = new GamepadEx();
    GamepadEx r1 = new GamepadEx();
    GamepadEx u1 = new GamepadEx();
    GamepadEx d1 = new GamepadEx();

    int randomizationResult;//0==left   1==center   2==right


    int yMod = 1;
    double headingMod = 0;

    double flipperScore = 0.32;
    double flipperIntake = 0.86;
    double flipperHold = 0.6;

    double ppp_up = 0.925;
    double ppp_down = 0.35;

    final Pose2d startBlueFront = new Pose2d(-36, 63.9375, Math.toRadians(90));
    final Pose2d startRedFront = new Pose2d(-36, -63.9375, Math.toRadians(-90));
    final Pose2d startBlueBack = new Pose2d(12, 63.9375, Math.toRadians(90));
    final Pose2d startRedBack = new Pose2d(12, -63.9375, Math.toRadians(-90));
    Pose2d finalStart = new Pose2d(0, 0, 0);
    final Pose2d backdropPos1 = new Pose2d(62, 42, 0);
    final Pose2d backdropPos2 = new Pose2d(62, 37, 0);
    final Pose2d backdropPos3 = new Pose2d(62, 27, 0);
    final Pose2d backdropPos4 = new Pose2d(62, -29, 0);
    final Pose2d backdropPos5 = new Pose2d(62, -36, 0);
    final Pose2d backdropPos6 = new Pose2d(62, -42, 0);
    Pose2d finalBackdropPos = new Pose2d(0, 0, 0);
    final Vector2d spikePosRedFront = new Vector2d(-36, -36);
    final Vector2d spikePosBlueFront = new Vector2d(-36, 36);
    final Vector2d spikePosRedBack = new Vector2d(12 + 8, -38);
    final Vector2d spikePosBlueBack = new Vector2d(12 + 8, 31.5);
    Vector2d finalSpikePos;

    Vector2d parkClose;
    Vector2d parkFar;


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
            l1.updateButton(gamepad1.dpad_left);
            r1.updateButton(gamepad1.dpad_right);
            u1.updateButton(gamepad1.dpad_up);
            d1.updateButton(gamepad1.dpad_down);

            if (lockedIn) {
                telemetry.addData("Press A to back out", "");
            } else {
                telemetry.addData("Press A to lock in decision\n" +
                        "Press B to change team\n" +
                        "Press X to change side\n" +
                        "Press Y to change parking location\n" +
                        "Press DPad Left to change the path\n" +
                        "Press DPad Up and Down to change the starting delay", "");
            }
            if (Team() == 0) {
                telemetry.addData("Team", "Blue");
            } else {
                telemetry.addData("Team", "Red");
            }
            if (Side() == 0) {
                telemetry.addData("Side", "FrontStage");
                telemetry.addData("Finish After Purple", finishEarly);
            } else {
                telemetry.addData("Side", "BackStage");
            }
            if (Park() == 0) {
                telemetry.addData("Park", "Far");
            } else {
                telemetry.addData("Park", "Close");
            }
            if (Path() == 0) {
                telemetry.addData("Path", "Rigging");
            } else {
                telemetry.addData("Path", "Stage Door");
            }
            telemetry.addData("Start Delay (seconds)", startDelay);

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
                if (l1.isPressed()) {
                    path++;
                }
                if (r1.isPressed() && Side() == 0) {
                    finishEarly = !finishEarly;
                }
                if (u1.isPressed() && startDelay < MAX_DELAY - 0.5) {
                    startDelay += 0.5;
                }
                if (d1.isPressed() && startDelay > 0) {
                    startDelay -= 0.5;
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
                randomizationResult = -1;
                finalBackdropPos = backdropPos1;
            } else if (pipelineBlue.getAnalysis() == ColorDetection.BlueDeterminationPipeline.TeamElementPosition.CENTER) {
                randomizationResult = 0;
                finalBackdropPos = backdropPos2;
            } else if (pipelineBlue.getAnalysis() == ColorDetection.BlueDeterminationPipeline.TeamElementPosition.RIGHT) {
                randomizationResult = 1;
                finalBackdropPos = backdropPos3;
            }
        } else {
            if (pipelineRed.getAnalysis() == ColorDetection.RedDeterminationPipeline.TeamElementPosition.LEFT) {
                randomizationResult = 1;
                finalBackdropPos = backdropPos4;
            } else if (pipelineRed.getAnalysis() == ColorDetection.RedDeterminationPipeline.TeamElementPosition.CENTER) {
                randomizationResult = 0;
                finalBackdropPos = backdropPos5;
            } else if (pipelineRed.getAnalysis() == ColorDetection.RedDeterminationPipeline.TeamElementPosition.RIGHT) {
                randomizationResult = -1;
                finalBackdropPos = backdropPos6;
            }
        }
        camera.closeCameraDevice();
    }

    public void positionAnalysis() {
        if (Team() == 0 && Side() == 0) {//blue frontStage
            yMod = -1;
            headingMod = Math.PI;
            finalStart = startBlueFront;
            finalSpikePos = spikePosBlueFront;
        } else if (Team() == 0 && Side() == 1) {//blue backstage
            yMod = -1;
            headingMod = 0.0;
            finalStart = startBlueBack;
            finalSpikePos = spikePosBlueBack;
        } else if (Team() == 1 && Side() == 0) {//red frontStage
            yMod = 1;
            headingMod = Math.PI;
            finalStart = startRedFront;
            finalSpikePos = spikePosRedFront;
        } else if (Team() == 1 && Side() == 1) {//red backStage
            yMod = 1;
            headingMod = 0.0;
            finalStart = startRedBack;
            finalSpikePos = spikePosRedBack;
        }

        //all positions with y,x,or heading Mod assume red left starting
        parkClose = new Vector2d(60, -63 * yMod);
        parkFar = new Vector2d(60, -9.5 * yMod);
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
        } else {
            waitEx(startDelay * 1000);
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
    }

    private void autoPreloadedBlueFrontStage(@NonNull MecanumDrive drive) {
        drive.ppp.setPosition(ppp_down);
        if (randomizationResult == -1) {//left
            if (Path() == 0) {//rigging
                drive.ppp.setPosition(ppp_down);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(3.5, 0)), Math.PI), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                drive.ppp.setPosition(ppp_up);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(drive.pose.position.plus(new Vector2d(-1, 0)), drive.pose.heading)
                        .build()
                );
                drive.updatePoseEstimate();
                if (finishEarly) {
                    waitEx(1000000);
                }
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(-48, 48, Math.PI / 2), Math.PI / 2)
                        .splineToSplineHeading(new Pose2d(new Vector2d(-24, 60), 0), 0)
                        .splineToSplineHeading(new Pose2d(new Vector2d(36, 54), Math.toRadians(-7)), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                park(new MecanumDrive(hardwareMap, new Pose2d(drive.pose.position.plus(new Vector2d(0, 8)), 0)));
            } else {//stage door

            }
        } else if (randomizationResult == 0) {//center
            if (Path() == 0) {//rigging
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(0, -1)), Math.PI / 2), -Math.PI / 2)
                        .build()
                );
                drive.updatePoseEstimate();
                drive.ppp.setPosition(ppp_up);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(drive.pose.position.plus(new Vector2d(0, 1)), drive.pose.heading)
                        .build()
                );
                drive.updatePoseEstimate();
                if (finishEarly) {
                    waitEx(1000000);
                }
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(new Vector2d(-24, 60), 0), 0)
                        .splineToSplineHeading(new Pose2d(new Vector2d(36, 60), 0), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                park(new MecanumDrive(hardwareMap, new Pose2d(drive.pose.position.plus(new Vector2d(0, 3)), 0)));
            } else {//stage door
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(finalStart.position.plus(new Vector2d(0, -5)), finalStart.heading)
                        .splineToLinearHeading(new Pose2d(-39, 13, -Math.PI / 2), -Math.PI / 2)
                        .build()
                );
                drive.updatePoseEstimate();
                drive.ppp.setPosition(ppp_up);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(drive.pose.position.plus(new Vector2d(0, -1)), drive.pose.heading)
                        .build()
                );
                drive.updatePoseEstimate();
                if (finishEarly) {
                    waitEx(1000000);
                }
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToConstantHeading(new Vector2d(-39, 10))
                        .strafeToLinearHeading(new Vector2d(-36, 10), 0)
                        .strafeToLinearHeading(new Vector2d(36, 12), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                park(drive);
            }
        } else {//right
            if (Path() == 0) {//rigging
                drive.ppp.setPosition(ppp_down);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(false)
                        .strafeToLinearHeading(finalSpikePos.plus(new Vector2d(-3.5, -6)), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                drive.ppp.setPosition(ppp_up);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(drive.pose.position.plus(new Vector2d(1, 0)), drive.pose.heading)
                        .build()
                );
                drive.updatePoseEstimate();
                if (finishEarly) {
                    waitEx(1000000);
                }
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(false)
                        .strafeToLinearHeading(new Vector2d(-36, 60), Math.PI / 4)
                        .strafeToLinearHeading(new Vector2d(-24, 60), 0)
                        .strafeToLinearHeading(new Vector2d(36, 58), Math.toRadians(-2))
                        .build()
                );
                drive.updatePoseEstimate();
                park(new MecanumDrive(hardwareMap, new Pose2d(drive.pose.position.plus(new Vector2d(0, 4)), 0)));
            }
        }
    }

    private void autoPreloadedBlueBackStage(@NonNull MecanumDrive drive) {
        drive.ppp.setPosition(ppp_down);
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(finalStart.position.x, finalStart.position.y - 5), finalStart.heading)
                .splineToSplineHeading(finalBackdropPos, 0)
                .build()
        );
        drive.updatePoseEstimate();
        scoreBackdrop(drive);
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(-9.5 * randomizationResult, yMod * 11 * ((randomizationResult + 1) % 2))), headingMod), headingMod - Math.PI)
                .build()
        );
        drive.ppp.setPosition(ppp_up);
        park(drive);
    }

    private void autoPreloadedRedFrontStage(@NonNull MecanumDrive drive) {
        drive.ppp.setPosition(ppp_down);
        if (randomizationResult == -1) {//left
            if (Path() == 0) {//rigging
                drive.ppp.setPosition(ppp_down);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(3.5, 0)), Math.PI), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                drive.ppp.setPosition(ppp_up);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(drive.pose.position.plus(new Vector2d(-1, 0)), drive.pose.heading)
                        .build()
                );
                drive.updatePoseEstimate();
                if (finishEarly) {
                    waitEx(1000000);
                }
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(-48, -48, -Math.PI / 2), -Math.PI / 2)
                        .splineToSplineHeading(new Pose2d(new Vector2d(-24, -60), 0), 0)
                        .splineToSplineHeading(new Pose2d(new Vector2d(36, -54), Math.toRadians(7)), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                park(new MecanumDrive(hardwareMap, new Pose2d(drive.pose.position.plus(new Vector2d(0, -8)), 0)));
            } else {//stage door

            }
        } else if (randomizationResult == 0) {//center
            if (Path() == 0) {//rigging
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(0, -2)), -Math.PI / 2), -Math.PI / 2)
                        .build()
                );
                drive.updatePoseEstimate();
                drive.ppp.setPosition(ppp_up);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(drive.pose.position.plus(new Vector2d(0, -1)), drive.pose.heading)
                        .build()
                );
                drive.updatePoseEstimate();
                if (finishEarly) {
                    waitEx(1000000);
                }
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(new Vector2d(-28, -60), 0), 0)
                        .splineToSplineHeading(new Pose2d(new Vector2d(36, -60), Math.toRadians(7)), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                park(new MecanumDrive(hardwareMap, new Pose2d(drive.pose.position.plus(new Vector2d(0, -14)), Math.toRadians(15))));
            } else {//stage door
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(finalStart.position.plus(new Vector2d(0, 5)), finalStart.heading)
                        .splineToLinearHeading(new Pose2d(-39, -13, Math.PI / 2), Math.PI / 2)
                        .build()
                );
                drive.updatePoseEstimate();
                drive.ppp.setPosition(ppp_up);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(drive.pose.position.plus(new Vector2d(0, 1)), drive.pose.heading)
                        .build()
                );
                drive.updatePoseEstimate();
                if (finishEarly) {
                    waitEx(1000000);
                }
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToConstantHeading(new Vector2d(-39, -10))
                        .strafeToLinearHeading(new Vector2d(-36, -10), 0)
                        .strafeToLinearHeading(new Vector2d(36, -12), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                park(drive);
            }
        } else {//right
            if (Path() == 0) {//rigging
                drive.ppp.setPosition(ppp_down);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(false)
                        .strafeToLinearHeading(finalSpikePos.plus(new Vector2d(-3.5, 6)), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                drive.ppp.setPosition(ppp_up);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(drive.pose.position.plus(new Vector2d(1, 0)), drive.pose.heading)
                        .build()
                );
                drive.updatePoseEstimate();
                if (finishEarly) {
                    waitEx(1000000);
                }
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(false)
                        .strafeToLinearHeading(new Vector2d(-36, -60), -Math.PI / 4)
                        .strafeToLinearHeading(new Vector2d(-24, -60), 0)
                        .strafeToLinearHeading(new Vector2d(36, -58), Math.toRadians(2))
                        .build()
                );
                drive.updatePoseEstimate();
                park(new MecanumDrive(hardwareMap, new Pose2d(drive.pose.position.plus(new Vector2d(0, -4)), 0)));
            }
        }
    }

    private void autoPreloadedRedBackStage(@NonNull MecanumDrive drive) {
        drive.ppp.setPosition(ppp_down);
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(finalStart.position.x, finalStart.position.y + 5), finalStart.heading)
                .splineToSplineHeading(finalBackdropPos, 0)
                .build()
        );
        drive.updatePoseEstimate();
        scoreBackdrop(drive);
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(-12 * randomizationResult, yMod * 10 * ((randomizationResult + 1) % 2))), headingMod), headingMod - Math.PI)
                .build()
        );
        drive.ppp.setPosition(ppp_up);
        park(drive);
    }

    public void park(@NonNull MecanumDrive drive) {
        if (Park() == 0) {//Far
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(-12, 0)), 0), 0)
                    .splineToSplineHeading(new Pose2d(parkFar, 0), 0)
                    .build()
            );
        } else if (Park() == 1) {//Close
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(-12, 0)), 0), 0)
                    .splineToSplineHeading(new Pose2d(parkClose, 0), 0)
                    .build()
            );
        }
    }


    private void waitEx(double milliseconds) {
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        time.reset();
        while (time.time(TimeUnit.MILLISECONDS) < milliseconds && opModeIsActive() && !isStopRequested()) {
        }
    }

    private void scoreBackdrop(@NonNull MecanumDrive drive) {
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


}
