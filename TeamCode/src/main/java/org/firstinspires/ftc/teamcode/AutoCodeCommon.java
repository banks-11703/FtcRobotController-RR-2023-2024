package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

    int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    private double desiredTagRange = 0;
    private double desiredTagBearing = 0;
    private double desiredTagYaw = 0;

    ElapsedTime timeSinceAprilTag = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime timeSinceSeen = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

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

    final Pose2d startRedLeft = new Pose2d(-36, -63.9375, Math.toRadians(-90));
    final Pose2d startRedRight = new Pose2d(36, -63.9375, Math.toRadians(-90));
    final Pose2d startBlueLeft = new Pose2d(36, 63.9375, Math.toRadians(90));
    final Pose2d startBlueRight = new Pose2d(-36, 63.9375, Math.toRadians(90));
    Pose2d finalStart = new Pose2d(0, 0, 0);
    Pose2d updatedPose = new Pose2d(0, 0, 0);
    final Vector2d backdropPos1 = new Vector2d(25, 42);
    final Vector2d backdropPos2 = new Vector2d(25, 36);
    final Vector2d backdropPos3 = new Vector2d(25, 30);
    final Vector2d backdropPos4 = new Vector2d(25, -30);
    final Vector2d backdropPos5 = new Vector2d(25, -36);
    final Vector2d backdropPos6 = new Vector2d(25, -42);
    Vector2d finalBackdropPos = new Vector2d(0, 0);

    Vector2d parkLower;
    Vector2d parkUpper;
    Vector2d scoreClose;
    Vector2d scoreMiddle;
    Vector2d scoreFar;
    Vector2d spikePosUpper;
    Vector2d spikePosLower;


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
                telemetry.clearAll();
                lockedIn = !lockedIn;
            }

            telemetry.update();
        }

        if (Team() == 0) {
            if (pipelineBlue.getAnalysis() == ColorDetection.BlueDeterminationPipeline.TeamElementPosition.LEFT) {
                randomizationResult = 0;
                DESIRED_TAG_ID = 1;
                finalBackdropPos = backdropPos1;
            } else if (pipelineBlue.getAnalysis() == ColorDetection.BlueDeterminationPipeline.TeamElementPosition.CENTER) {
                randomizationResult = 1;
                DESIRED_TAG_ID = 2;
                finalBackdropPos = backdropPos2;
            } else if (pipelineBlue.getAnalysis() == ColorDetection.BlueDeterminationPipeline.TeamElementPosition.RIGHT) {
                randomizationResult = 2;
                DESIRED_TAG_ID = 3;
                finalBackdropPos = backdropPos3;
            }
        } else {
            if (pipelineRed.getAnalysis() == ColorDetection.RedDeterminationPipeline.TeamElementPosition.LEFT) {
                randomizationResult = 0;
                DESIRED_TAG_ID = 4;
                finalBackdropPos = backdropPos4;
            } else if (pipelineRed.getAnalysis() == ColorDetection.RedDeterminationPipeline.TeamElementPosition.CENTER) {
                randomizationResult = 1;
                DESIRED_TAG_ID = 5;
                finalBackdropPos = backdropPos5;
            } else if (pipelineRed.getAnalysis() == ColorDetection.RedDeterminationPipeline.TeamElementPosition.RIGHT) {
                randomizationResult = 2;
                DESIRED_TAG_ID = 6;
                finalBackdropPos = backdropPos6;
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
        spikePosUpper = new Vector2d(-36 + xMod, -14 * yMod);
        spikePosLower = new Vector2d(-36 + xMod, -32 * yMod);
    }

    public void buildTrajectories(MecanumDrive drive) {

    }

    public void scorePreloadedFloor(MecanumDrive drive) {
        if (finalStart.position.x == 0) {
            waitEx(1000000);
        }


        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(finalStart.position.x, finalStart.position.y + 5 * yMod), finalStart.heading)
                .build()
        );
        drive.updatePoseEstimate();
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(90 * randomizationResult + 90))
                .build()
        );
        drive.updatePoseEstimate();
        waitEx(1000);

        if (randomizationResult == 1) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(spikePosUpper, Math.toRadians(-90 * randomizationResult + 180 + headingMod))
                    .build()
            );
            drive.updatePoseEstimate();
        } else {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(spikePosLower, Math.toRadians(-90 * randomizationResult + 180 + headingMod))
                    .build());
            drive.updatePoseEstimate();
        }
        waitEx(100000);
    }

    public void driveToBackStage(MecanumDrive drive) {
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(drive.pose.position.x, -12 * yMod), Math.toRadians(0))
                .build()
        );
        drive.updatePoseEstimate();

        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(36, -12 * yMod), new Rotation2d(1.0, 0.0))
                .build()
        );
        drive.updatePoseEstimate();
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(finalBackdropPos, 0)
                .build()
        );
        drive.updatePoseEstimate();
    }

    public void scorePreloadedBackdrop(MecanumDrive drive) {


    }

    public void park(MecanumDrive drive) {}

//    /**
//     * grabServoPos:
//     * <p>
//     * open = 0
//     * <p>
//     * closed = 1
//     * <p>
//     * dropServoPos:
//     * <p>
//     * up = 1
//     * <p>
//     * down = 0
//     */
//    private void moveServos(double servoLeft, double servoRight, double dropServoPos, MecanumDrive drive) {
//        drive.intakeServoL.setPosition(servoLeft);
//        drive.intakeServoR.setPosition(servoRight);
//        drive.dropServo.setPosition(dropServoPos);
//    }

    private void waitEx(double milliseconds) {
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        time.reset();
        while (time.time(TimeUnit.MILLISECONDS) < milliseconds && opModeIsActive() && !isStopRequested()) {}
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

        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        timeSinceAprilTag.reset();

        while (rangeError == 0 && headingError == 0 && yawError == 0 && timeSinceAprilTag.time(TimeUnit.MILLISECONDS) < 3000 && opModeIsActive() && !isStopRequested()) {
            targetFound = false;
            desiredTag = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
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
                    }
                }
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
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
                strafe = 0;
                turn = 0;
            }


            moveRobot(forward, strafe, turn, drive);
//            updatedPose =
            sleep(10);
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
    public void moveRobot(double x, double y, double yaw, MecanumDrive drive) {
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

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
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
     * This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (opModeIsActive() && !isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
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

//    public void goToAprilTag(MecanumDrive drive) {
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        boolean done = false;
//        while (!done && !isStopRequested() && opModeIsActive()) {
//            double xPower = 0;
//            double yPower = 0;
//            double rotationPower = 0;
//            for (AprilTagDetection detection : currentDetections) {
//                if (detection.id == targetId) {
//                    xPower = detection.ftcPose.x / aprilTagXMod;
//                    yPower = (detection.ftcPose.y - 10) / aprilTagYMod;
//                    rotationPower = detection.ftcPose.yaw / aprilTagHeadingMod;
//                    if (Math.abs(detection.ftcPose.x) <= 1 &&
//                            Math.abs(detection.ftcPose.y - 10) <= 1 &&
//                            Math.abs(detection.ftcPose.yaw - 90) <= 5) {
//                        done = true;
//                    }
//                }
//            }
//            drive.setDrivePowers(new PoseVelocity2d(
//                    new Vector2d(
//                            yPower,
//                            xPower
//                    ),
//                    rotationPower
//            ));
//
//            drive.updatePoseEstimate();
//
//
//        }
//
//    }

}
