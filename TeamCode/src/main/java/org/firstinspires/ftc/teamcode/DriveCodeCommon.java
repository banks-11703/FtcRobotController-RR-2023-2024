package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
//@TeleOp
//@Disabled
public class DriveCodeCommon extends LinearOpMode {

    // Adjust these numbers to suit your robot.
    public double DESIRED_DISTANCE = 8; //  this is how close the camera should get to the target (inches)
    public double DESIRED_BEARING = 0;
    public double DESIRED_YAW = -4;

    public double DISTANCE_F = 0.075;
    public double BEARING_F = 0.2;
    public double YAW_F = 0.10;

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    public double SPEED_GAIN = 0.04;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public double STRAFE_GAIN = 0.04;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public double TURN_GAIN = 0.02;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.3;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.3;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.2;   //  Clip the turn speed to this max value (adjust for your robot)

    private final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    public static int DESIRED_TAG_ID = 5;     // Choose the tag you want to approach or set to -1 for ANY tag.
    public static int DESIRED_TAG_ID2 = 2;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    private double desiredTagRange = 0;
    private double desiredTagBearing = 0;
    private double desiredTagYaw = 0;
    boolean targetFound = false;    // Set to true when an AprilTag target is detected
    double forward = 0;        // Desired forward power/speed (-1 to +1)
    double strafe = 0;        // Desired strafe power/speed (-1 to +1)
    double turn = 0;        // Desired turning power/speed (-1 to +1)
    ElapsedTime timeSinceSeen = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public double latchClosed = 0;
    public double latchOpen1 = 0.5;
    public double latchOpen2 = 1;


    //    public static double[] latch = {latchClosed,latchOpen1,latchOpen2};
    double[] latch = {0.74, 0.79, 0.9};
    public static double flipperscore = 0.32;
    public static double flipperintake = 0.86;
    public static double flipperadjust = 0.2;
    int backSpeed = -200;
    public static int forwardSpeed = 2300;
    public static double dropServoUp = 1;
    public static double dropServoDown = 0.05;
    public static double planeClosed = 0.35;
    public static double planeOpen = 0.75;
    int planeTargetPos = 0;
    ElapsedTime planeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    boolean liftBusy = false;
    boolean slamLift = false;
    int[] liftTargetPos = {0, 100, 200, 300};

    double liftModSum = 0;
    int finalLiftPos = 0;
    double driveSpeedMod = 1;

    double DriveSpeedMod() {
        if (gamepad1.left_trigger + gamepad1.right_trigger > 0.1) {
            return 2;
        } else {
            return 1;
        }
    }

    public static double forwardIntakeSpeed = -1;

    GamepadEx a1 = new GamepadEx();
    GamepadEx x1 = new GamepadEx();//intake
    GamepadEx b1 = new GamepadEx();//intake out
    GamepadEx x2 = new GamepadEx();//outtake
    GamepadEx a2 = new GamepadEx();//liftToZero

    GamepadEx b2 = new GamepadEx();//adjust
    GamepadEx dpadL1 = new GamepadEx();//planeLauncher
    GamepadEx dpadD1 = new GamepadEx();// stack lift
    GamepadEx rBumper1 = new GamepadEx();

    GamepadEx y2 = new GamepadEx();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void Initialization(MecanumDrive drive) {
        telemetry.update();
        drive.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftLift.setTargetPosition(liftTargetPos[0]);
        drive.rightLift.setTargetPosition(liftTargetPos[0]);
        drive.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        drive.planeLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drive.planeLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Initialize the Apriltag Detection process
        initAprilTag();

        if (USE_WEBCAM) setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        waitForStart();
    }

    public void updateButtons(MecanumDrive drive) {
        a1.updateButton(gamepad1.a);
        x1.updateButton(gamepad1.x);
        x2.updateButton(gamepad2.x);
        b1.updateButton(gamepad1.b);
        a2.updateButton(gamepad2.a);
        b2.updateButton(gamepad2.b);
        y2.updateButton(gamepad2.y);
        dpadL1.updateButton(gamepad1.dpad_left);
//        rBumper1.updateButton(gamepad1.right_bumper);
//        dpadD1.updateButton(gamepad1.dpad_down);
    }

    public void rawDriving(MecanumDrive drive) {
        drive.frontLeft.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * DriveSpeedMod());
        drive.frontRight.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * DriveSpeedMod());
        drive.backRight.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * DriveSpeedMod());
        drive.backLeft.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * DriveSpeedMod());

    }

    public void aprilTagDriving(MecanumDrive drive) {
        targetFound = false;
        desiredTag = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID || detection.id == DESIRED_TAG_ID2)) {
                    // Yes, we want to use this tag.
                    if (gamepad1.left_bumper) {
                        timeSinceSeen.reset();
                    }
                    targetFound = true;
                    desiredTag = detection;
                    desiredTagRange = desiredTag.ftcPose.range;
                    desiredTagBearing = desiredTag.ftcPose.bearing;
                    desiredTagYaw = desiredTag.ftcPose.yaw;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
        }

        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
        if ((gamepad1.left_bumper && targetFound)) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (desiredTagRange - DESIRED_DISTANCE) + DISTANCE_F;
            double headingError;
            double yawError;
            if (!(gamepad1.left_bumper && targetFound)) {
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

//                forward = DISTANCE_F;
//                turn = YAW_F;
//                strafe = BEARING_F;

            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", forward, strafe, turn);
        } else {

            // drive using manual POV Joystick mode.  Slow things down to make the robot more controllable.
            forward = -gamepad1.left_stick_y / DriveSpeedMod();  // Reduce drive rate to 50%.
            strafe = -gamepad1.left_stick_x / DriveSpeedMod();  // Reduce strafe rate to 50%.
            turn = -gamepad1.right_stick_x / DriveSpeedMod();  // Reduce turn rate to 33%.
            telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", forward, strafe, turn);
        }


        // Apply desired axes motions to the drivetrain.
        telemetry.addData("Forward Power", forward);
        telemetry.addData("Strafe Power", strafe);
        telemetry.addData("Turn Power", turn);
        moveRobot(forward, strafe, turn, rBumper1.isToggled(), drive);
        if ((int) getRuntime() % 10 == 0) {
            telemetry.clearAll();
        }
    }

    public void pidDriving(MecanumDrive drive) {
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));
    }

    public void intake(MecanumDrive drive) {//1
        if (b1.isHeld()) {
            drive.intake.setPower(1);
        } else if (x1.isToggled()) {
            drive.intake.setPower(forwardIntakeSpeed);
        } else {
            drive.intake.setPower(0);
        }
        if (gamepad1.y) {
            drive.intakeServoL.setPosition(0);
            drive.intakeServoR.setPosition(1);
        } else {
            drive.intakeServoL.setPosition(1);
            drive.intakeServoR.setPosition(0);
        }
        if (dpadD1.isToggled()) {
            drive.dropServo.setPosition(dropServoDown);
        } else {
            drive.dropServo.setPosition(dropServoUp);
        }
    }

    public void lift(MecanumDrive drive) {//2
        if (gamepad2.dpad_down) slamLift = true;
        if (slamLift && ((gamepad2.right_trigger + gamepad2.left_trigger >= 0.1) || gamepad2.a)) {
            slamLift = false;
        } else if (slamLift) {
            drive.leftLift.setPower(-0.95);
            drive.rightLift.setPower(-0.95);
        }
        drive.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (drive.leftLift.getCurrentPosition() < 1930 && drive.rightLift.getCurrentPosition() > 0) {
            drive.leftLift.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
            drive.rightLift.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        } else if (drive.leftLift.getCurrentPosition() < 1930) {
            drive.leftLift.setPower(gamepad2.right_trigger);
            drive.rightLift.setPower(gamepad2.right_trigger);
        } else {
            drive.leftLift.setPower(-gamepad2.left_trigger);
            drive.rightLift.setPower(-gamepad2.left_trigger);
        }

    }


    public void outake(MecanumDrive drive) {//2
        if (x2.isHeld()) {
            drive.flipper.setPosition(flipperscore);
        } else if (b2.isHeld()) {
            drive.flipper.setPosition(flipperadjust);
        } else {
            drive.flipper.setPosition(flipperintake);
        }
    }

    public void launcher(MecanumDrive drive) {

        if(dpadL1.isToggled()){
            drive.launchLatch.setPosition(0);
        }else {
            drive.launchLatch.setPosition(1);
        }
        //2

//        dpadL1.updateButton(gamepad1.dpad_left);
//        if(dpadL1.isHeld()) {
//            drive.launchLatch.setPosition(planeOpen);
//        } else {
//            drive.launchLatch.setPosition(planeClosed);
//        }
//        switch (dpadL1.getCycle()) {
//            case 0:
//                drive.planeLauncher.setVelocity(0);
//                drive.launchLatch.setPosition(planeClosed);
////                drive.planeLauncher.setTargetPosition(planeTargetPos);
////                drive.planeLauncher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                drive.planeLauncher.setPower(1);
//                dpadL1.updateButton(gamepad1.dpad_left);
//                planeTimer.reset();
//                break;
//            case 1:
//                drive.planeLauncher.setPower(0);
//                drive.planeLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                if (planeTimer.milliseconds() > 500) {
//                    planeTimer.reset();
//                    dpadL1.setToggle(dpadL1.getCycle() + 1);
//                }
//                break;
//            case 2:
//                drive.planeLauncher.setVelocity(backSpeed);
//                if (planeTimer.milliseconds() > 1000) {
//                    planeTimer.reset();
//                    dpadL1.setToggle(dpadL1.getCycle() + 1);
//                }
//                break;
//            case 3:
//                drive.planeLauncher.setVelocity(forwardSpeed);
//                if (Math.abs(drive.planeLauncher.getVelocity() - forwardSpeed) < 20 || gamepad1.dpad_left) {
//                    planeTimer.reset();
//                    dpadL1.setToggle(dpadL1.getCycle() + 1);
//                }
//                break;
//            case 4:
//                drive.launchLatch.setPosition(planeOpen);
//                planeTargetPos = drive.planeLauncher.getCurrentPosition();
//                if (planeTimer.milliseconds() > 500) {
//                    dpadL1.setToggle(0);
//                }
//                break;
//        }
    }

    public void telemetry(MecanumDrive drive) {
//        telemetry.addData("Launcher Velocity: ", drive.planeLauncher.getVelocity());
        telemetry.addData("Servo Pos: ", drive.launchLatch.getPosition());
//        telemetry.addData("Launcher Power; ", drive.planeLauncher.getPower());
        telemetry.addData("Lift encoder Left", drive.leftLift.getCurrentPosition());
        telemetry.addData("Lift encoder Right", drive.rightLift.getCurrentPosition());
        telemetry.update();
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
    public void moveRobot(double x, double y, double yaw, boolean reversedDriving, MecanumDrive drive) {
        // Calculate wheel powers.
        int powerMod = 1;
        if (reversedDriving) {
            powerMod = -1;
        }
        double leftFrontPower = powerMod * (x - y - yaw);
        double rightFrontPower = powerMod * (x + y + yaw);
        double leftBackPower = powerMod * (x + y - yaw);
        double rightBackPower = powerMod * (x - y + yaw);

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
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /**
     * Manually set the camera gain and exposure.
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
}
