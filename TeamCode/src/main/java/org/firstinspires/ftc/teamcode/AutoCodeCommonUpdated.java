package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;

@Autonomous
@Disabled
public class AutoCodeCommonUpdated extends LinearOpMode {

    OpenCvCamera camera;
    public ColorDetection.RedDeterminationPipeline pipelineRed;
    public ColorDetection.BlueDeterminationPipeline pipelineBlue;


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

    int side = 1;

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

    boolean score = true;
    boolean finishEarly = false;

    double startDelay = 0.0;
    double scoreDelay = 0.0;

    GamepadEx a1 = new GamepadEx();
    GamepadEx b1 = new GamepadEx();
    GamepadEx x1 = new GamepadEx();
    GamepadEx y1 = new GamepadEx();
    GamepadEx l1 = new GamepadEx();
    GamepadEx r1 = new GamepadEx();
    GamepadEx u1 = new GamepadEx();
    GamepadEx d1 = new GamepadEx();
    GamepadEx yrPlus = new GamepadEx();
    GamepadEx yrMinus = new GamepadEx();
    GamepadEx ylPlus = new GamepadEx();
    GamepadEx ylMinus = new GamepadEx();

    /**
     * -1==backstage
     * 0==center
     * 1==frontstage
     */
    int randomizationResult;


    int yMod = 1;
    double headingMod = 0;

    double flipperScore = 0.51;
    double flipperIntake = 0.98;

    double grabyl_in = 0.41;
    double grabyl_out = 1;
    double grabyr_in = 0.9;
    double grabyr_out = 0;
    double grabyMid = 0.66;

    final Pose2d startBlueFront = new Pose2d(-36, 63.9375, Math.toRadians(90));
    final Pose2d startRedFront = new Pose2d(-36, -63.9375, Math.toRadians(-90));
    final Pose2d startBlueBack = new Pose2d(12, 63.9375, Math.toRadians(90));
    final Pose2d startRedBack = new Pose2d(12, -63.9375, Math.toRadians(-90));
    Pose2d finalStart = new Pose2d(0, 0, 0);
    final Pose2d backdropPos1 = new Pose2d(59.5, 42, 0);
    final Pose2d backdropPos2 = new Pose2d(59.5, 35, 0);
    final Pose2d backdropPos3 = new Pose2d(59.5, 27, 0);
    final Pose2d backdropPos4 = new Pose2d(59.5, -29, 0);
    final Pose2d backdropPos5 = new Pose2d(59.5, -36, 0);
    final Pose2d backdropPos6 = new Pose2d(59.5, -42, 0);
    Pose2d finalBackdropPos = new Pose2d(0, 0, 0);
    final Vector2d spikePosRedFront = new Vector2d(-36, -36);
    final Vector2d spikePosBlueFront = new Vector2d(-36, 36);
    final Vector2d spikePosRedBack = new Vector2d(12 + 8, -38);
    final Vector2d spikePosBlueBack = new Vector2d(12 + 8, 31.5);
    Vector2d finalSpikePos;
    final Pose2d stackPosRedClose = new Pose2d(-62, -36, Math.toRadians(0));
    final Pose2d stackPosRedMedium = new Pose2d(-62, -24, Math.toRadians(0));
    final Pose2d stackPosRedFar = new Pose2d(-64, -20.8, Math.toRadians(9));
    final Pose2d stackPosBlueFar = new Pose2d(-62, 12, Math.toRadians(0));
    final Pose2d stackPosBlueMedium = new Pose2d(-62, 24, Math.toRadians(0));
    final Pose2d stackPosBlueClose = new Pose2d(-60, 35, Math.toRadians(0));
    Pose2d finalStackPos;

    Vector2d parkClose;
    Vector2d parkFar;

    VelConstraint slowSpeed = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(20.0),
            new AngularVelConstraint(Math.PI / 2)
    ));

    AccelConstraint slowAccel = new ProfileAccelConstraint(-10.0, 25.0);


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
            yrPlus.updateButton(-gamepad1.right_stick_y > 0);
            yrMinus.updateButton(-gamepad1.right_stick_y < 0);
            ylPlus.updateButton(-gamepad1.left_stick_y > 0);
            ylMinus.updateButton(-gamepad1.left_stick_y < 0);

            if (lockedIn) {
                telemetry.addData("Press A to back out", "");
            } else {
                telemetry.addData("Press DPad Up to choose whether to perform extra actions\n" +
                        "Move Joysticks Up and Down to change the starting delay", "");
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
            telemetry.addData("Finish After Purple", finishEarly);
            if (!finishEarly) {
                telemetry.addData("Extra Scoring?", score);
            }
            if (Side() == 0) {
                if (Path() == 0) {
                    telemetry.addData("Path", "Rigging");
                } else {
                    telemetry.addData("Path", "Stage Door");
                }

                telemetry.addData("Score delay (seconds)", scoreDelay);
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
                if (r1.isPressed()) {

                }
                if (u1.isPressed()) {
                    score = !score;
                }
                if (d1.isPressed()) {
                    finishEarly = !finishEarly;
                }
                if (yrPlus.isPressed()) {
                    startDelay += 0.5;
                } else if (yrMinus.isPressed() && startDelay - 0.5 >= 0) {
                    startDelay -= 0.5;
                }
                if (ylPlus.isPressed()) {
                    scoreDelay += 0.5;
                } else if (ylMinus.isPressed() && scoreDelay - 0.5 >= 0) {
                    scoreDelay -= 0.5;
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
            finalStackPos = null;
        } else if (Team() == 0 && Side() == 1) {//blue backstage
            yMod = -1;
            headingMod = 0.0;
            finalStart = startBlueBack;
            finalSpikePos = spikePosBlueBack;
            finalStackPos = stackPosBlueFar;
        } else if (Team() == 1 && Side() == 0) {//red frontStage
            yMod = 1;
            headingMod = Math.PI;
            finalStart = startRedFront;
            finalSpikePos = spikePosRedFront;
            finalStackPos = null;
        } else if (Team() == 1 && Side() == 1) {//red backStage
            yMod = 1;
            headingMod = 0.0;
            finalStart = startRedBack;
            finalSpikePos = spikePosRedBack;
            finalStackPos = stackPosRedFar;
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
        purplePixel(false, drive);
        waitEx(500);
        drive.intake.setPower(0.4);
        if (randomizationResult == -1) {//left
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(1, -5.5)), Math.PI), 0)
                    .afterTime(0, intakeTraj(0, drive))
                    .afterTime(0, purplePixelTraj(grabyl_out, grabyr_out, drive))
                    .build()
            );
            drive.updatePoseEstimate();
            if (finishEarly) {
                waitEx(1000000);
            }
            if (score) {
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(false)
                        .strafeToLinearHeading(stackPosBlueClose.position.plus(new Vector2d(5, 0)), 0)
                        .afterDisp(0, purplePixelTraj(grabyMid, grabyMid, drive))
                        .strafeToLinearHeading(stackPosBlueClose.position, stackPosBlueClose.heading)
                        .build()
                );
                drive.updatePoseEstimate();
                intakeStack("frontStage", drive);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .afterDisp(5, purplePixelTraj(grabyl_out, grabyr_out, drive))
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(-40, 59, Math.toRadians(-3)), Math.toRadians(-3))
                        .splineToSplineHeading(new Pose2d(12, 59, Math.toRadians(-3)), Math.toRadians(-3))
                        .waitSeconds(scoreDelay)
                        .splineToSplineHeading(new Pose2d(24, 52, -Math.PI / 6), -Math.PI / 6)
                        .splineToSplineHeading(new Pose2d(finalBackdropPos.position.plus(new Vector2d(-8, 0)), Math.toRadians(0)), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                drive.intake.setPower(0);
                scoreBackdrop(50, drive);
                waitEx(500);
                drive.intake.setPower(-1);
                waitEx(1200);
                drive.intake.setPower(0);
                scoreBackdrop(150, drive);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(finalBackdropPos.position.plus(new Vector2d(-20, 0)), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                park(0, drive);
            } else {
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(-48, 48, Math.PI / 2), Math.PI / 2)
                        .splineToSplineHeading(new Pose2d(new Vector2d(-24, 60), 0), 0)
                        .splineToSplineHeading(new Pose2d(new Vector2d(36, 60), Math.toRadians(0)), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                park(0, drive);
            }
        } else if (randomizationResult == 0) {//center
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(0, 1.5)), Math.PI / 2), Math.PI / 2)
                    .afterTime(0, intakeTraj(0, drive))
                    .afterTime(0, purplePixelTraj(grabyl_out, grabyr_out, drive))
                    .setReversed(false)
                    .strafeToLinearHeading(finalSpikePos.plus(new Vector2d(0, 5)), drive.pose.heading)
                    .build()
            );
            drive.updatePoseEstimate();
            if (finishEarly) {
                waitEx(1000000);
            }
            if (score) {
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(stackPosBlueClose.position.plus(new Vector2d(5, 3.5)), 0)
                        .afterDisp(0, purplePixelTraj(grabyMid, grabyMid, drive))
                        .strafeToLinearHeading(stackPosBlueClose.position.plus(new Vector2d(-0.5, 3.5)), stackPosBlueClose.heading.plus(Math.toRadians(7)))
                        .build()
                );
                drive.updatePoseEstimate();
                intakeStack("frontStage", drive);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .afterDisp(5, purplePixelTraj(grabyl_out, grabyr_out, drive))
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(-40, 60, Math.toRadians(-3)), Math.toRadians(-3))
                        .splineToSplineHeading(new Pose2d(12, 58, Math.toRadians(-3)), Math.toRadians(-3))
                        .waitSeconds(scoreDelay)
                        .splineToSplineHeading(new Pose2d(24, 52, -Math.PI / 6), -Math.PI / 6)
                        .splineToSplineHeading(new Pose2d(finalBackdropPos.position.plus(new Vector2d(-8, -3)), Math.toRadians(-3)), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                drive.intake.setPower(0);
                scoreBackdrop(50, drive);
                waitEx(500);
                drive.intake.setPower(-1);
                waitEx(1200);
                drive.intake.setPower(0);
                scoreBackdrop(150, drive);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(finalBackdropPos.position.plus(new Vector2d(-20, -5)), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                park(0, drive);
            } else {
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(new Vector2d(-28, 60), 0), 0)
                        .splineToSplineHeading(new Pose2d(new Vector2d(36, 60), Math.toRadians(0)), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                park(0, drive);
            }
        } else {//Right
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .strafeToLinearHeading(drive.pose.position.plus(new Vector2d(0, -1)), Math.PI / 2)
                    .splineToLinearHeading(new Pose2d(finalSpikePos.plus(new Vector2d(-11.5, 2)), Math.PI / 2), Math.PI / 2, slowSpeed)
                    .afterDisp(0, purplePixelTraj(grabyl_out, grabyr_out, drive))
                    .afterDisp(1, intakeTraj(0, drive))
                    .setReversed(false)
                    .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(-4, 7)), 0), 0, slowSpeed, slowAccel)
                    .build()
            );
            drive.updatePoseEstimate();
            if (finishEarly) {
                waitEx(1000000);
            }
            if (score) {
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(stackPosBlueClose.position.plus(new Vector2d(5, 3.5)), Math.toRadians(0))
                        .afterDisp(0, purplePixelTraj(grabyMid, grabyMid, drive))
                        .strafeToLinearHeading(stackPosBlueClose.position.plus(new Vector2d(-1, 3.5)), stackPosBlueClose.heading.plus(Math.toRadians(5)))
                        .build()
                );
                drive.updatePoseEstimate();
                intakeStack("frontStage", drive);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .afterDisp(5, purplePixelTraj(grabyl_out, grabyr_out, drive))
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(-40, 58, Math.toRadians(-6)), Math.toRadians(-6))
                        .splineToSplineHeading(new Pose2d(12, 58, Math.toRadians(-6)), Math.toRadians(-6))
                        .waitSeconds(scoreDelay)
                        .splineToSplineHeading(new Pose2d(24, 52, -Math.PI / 6), -Math.PI / 6)
                        .splineToSplineHeading(new Pose2d(finalBackdropPos.position.plus(new Vector2d(-8, -6)), Math.toRadians(-1)), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                drive.intake.setPower(0);
                scoreBackdrop(50, drive);
                waitEx(500);
                drive.intake.setPower(-1);
                waitEx(1200);
                drive.intake.setPower(0);
                scoreBackdrop(150, drive);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(finalBackdropPos.position.plus(new Vector2d(-20, -4)), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                park(0, drive);

            } else {
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(false)
                        .strafeToLinearHeading(new Vector2d(-36, 60), Math.PI / 4)
                        .strafeToLinearHeading(new Vector2d(-24, 60), 0)
                        .strafeToLinearHeading(new Vector2d(36, 58), Math.toRadians(0))
                        .build()
                );
                drive.updatePoseEstimate();
                park(0, drive);
            }
        }
    }

    private void autoPreloadedBlueBackStage(@NonNull MecanumDrive drive) {
        purplePixel(false, drive);
        waitEx(500);
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .afterDisp(0, intakeTraj(0.4, drive))
                .strafeToLinearHeading(new Vector2d(finalStart.position.x, finalStart.position.y - 5), finalStart.heading)
                .afterDisp(0, intakeTraj(0.4, drive))
                .splineToSplineHeading(new Pose2d(finalBackdropPos.position.plus(new Vector2d(-7.5, 1.5)), Math.toRadians(-2)), 0)
                .build()
        );
        drive.updatePoseEstimate();
        scoreBackdrop(0, drive);
        if (score && !finishEarly) {
            if (randomizationResult == -1) {//Left
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(10.5, -2)), 0), -Math.PI)
                        .build()
                );
            } else if (randomizationResult == 0) {//Center
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(0, -9)), 0), -Math.PI)
                        .build()
                );
            } else if (randomizationResult == 1) {//Right
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(-10, 0)), 0), -Math.PI)
                        .build()
                );
            }
            drive.updatePoseEstimate();
            drive.intake.setPower(0);
            purplePixel(true, drive);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(40, 10), 0)
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(finalStackPos.position.plus(new Vector2d(10, 0)), finalStackPos.heading), Math.PI)
                    .waitSeconds(0.1)
                    .splineToSplineHeading(finalStackPos, Math.PI)
                    .build()
            );
            drive.updatePoseEstimate();
            intakeStack("backStage", drive);
            waitEx(100000);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .splineToSplineHeading(new Pose2d(-39, 10, 0), 0)
                    .splineToSplineHeading(new Pose2d(-36, 10, 0), 0)
                    .splineToSplineHeading(new Pose2d(12, 10, 0), 0)
                    .splineToSplineHeading(new Pose2d(36, 10, 0), 0)
                    .splineToSplineHeading(backdropPos5, 0)
                    .build()
            );
            drive.updatePoseEstimate();
            drive.intake.setPower(0);
            scoreBackdrop(drive);
        } else {
            if (Park() == 0) {//Far
                if (randomizationResult == -1) {//Left
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .setReversed(true)
                            .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(9.5, -2)), 0), -Math.PI)
                            .afterDisp(0, intakeTraj(0, drive))
                            .afterDisp(0, purplePixelTraj(grabyl_out, grabyr_out, drive))
                            .setReversed(false)
                            .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(-12, 0)), 0), 0)
                            .splineToSplineHeading(new Pose2d(parkFar, 0), 0)
                            .build()
                    );
                } else if (randomizationResult == 0) {//Center
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .setReversed(true)
                            .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(0, -7)), 0), -Math.PI)
                            .afterDisp(0, intakeTraj(0, drive))
                            .afterDisp(0, purplePixelTraj(grabyl_out, grabyr_out, drive))
                            .setReversed(false)
                            .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(-12, 0)), 0), 0)
                            .splineToSplineHeading(new Pose2d(parkFar, 0), 0)
                            .build()
                    );
                } else if (randomizationResult == 1) {//Right
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .setReversed(true)
                            .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(-11.5, 4)), 0), -Math.PI)
                            .afterDisp(0, intakeTraj(0, drive))
                            .afterDisp(0, purplePixelTraj(grabyl_out, grabyr_out, drive))
                            .setReversed(false)
                            .strafeToLinearHeading(finalSpikePos.plus(new Vector2d(-5, 4)), 0)
                            .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(0,1)), 0), 0)
                            .build()
                    );
                }
            } else {//Close
                if (randomizationResult == -1) {//Left
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .setReversed(true)
                            .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(9.5, -2)), 0), -Math.PI)
                            .afterDisp(0, intakeTraj(0, drive))
                            .afterDisp(0, purplePixelTraj(grabyl_out, grabyr_out, drive))
                            .setReversed(false)
                            .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(-12, 0)), 0), 0)
                            .splineToSplineHeading(new Pose2d(parkClose, 0), 0)
                            .build()
                    );
                } else if (randomizationResult == 0) {//Center
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .setReversed(true)
                            .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(0, -7)), 0), -Math.PI)
                            .afterDisp(0, intakeTraj(0, drive))
                            .afterDisp(0, purplePixelTraj(grabyl_out, grabyr_out, drive))
                            .setReversed(false)
                            .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(-12, 0)), 0), 0)
                            .splineToSplineHeading(new Pose2d(parkClose, 0), 0)
                            .build()
                    );
                } else if (randomizationResult == 1) {//Right
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .setReversed(true)
                            .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(-11.5, 4)), 0), -Math.PI)
                            .afterDisp(0, intakeTraj(0, drive))
                            .afterDisp(0, purplePixelTraj(grabyl_out, grabyr_out, drive))
                            .setReversed(false)
                            .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(-12, 0)), 0), 0)
                            .splineToSplineHeading(new Pose2d(parkClose, 0), 0)
                            .build()
                    );
                }
            }
        }
    }

    private void autoPreloadedRedFrontStage(@NonNull MecanumDrive drive) {
        purplePixel(false, drive);
        waitEx(500);
        drive.intake.setPower(0.4);
        if (randomizationResult == -1) {//Right
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(5, 5.5)), Math.PI), 0)
                    .afterTime(0, intakeTraj(0, drive))
                    .afterTime(0, purplePixelTraj(grabyl_out, grabyr_out, drive))
                    .setReversed(false)
                    .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(0, 5.5)), Math.PI), Math.PI)
                    .build()
            );
            drive.updatePoseEstimate();
            if (finishEarly) {
                waitEx(1000000);
            }
            if (score) {
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(stackPosRedClose.position.plus(new Vector2d(9, -2)), 0)
                        .afterDisp(0, purplePixelTraj(grabyMid, grabyMid, drive))
                        .strafeToLinearHeading(stackPosRedClose.position.plus(new Vector2d(2.5, -2)), Math.toRadians(-10))
                        .build()
                );
                drive.updatePoseEstimate();
                intakeStack("frontStage", drive);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .afterDisp(5, purplePixelTraj(grabyl_out, grabyr_out, drive))
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(-40, -60, Math.toRadians(-3)), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(12, -63, Math.toRadians(-3)), Math.toRadians(0))
                        .waitSeconds(scoreDelay)
                        .splineToSplineHeading(new Pose2d(finalBackdropPos.position.plus(new Vector2d(-6.5, -16)), Math.toRadians(-7)), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                drive.intake.setPower(0);
                scoreBackdrop(50, drive);
                waitEx(500);
                drive.intake.setPower(-1);
                waitEx(1200);
                drive.intake.setPower(0);
                scoreBackdrop(150, drive);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(finalBackdropPos.position.plus(new Vector2d(-20, -15)), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                park(2, drive);
            } else {
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(-48, -48, -Math.PI / 2), -Math.PI / 2)
                        .splineToSplineHeading(new Pose2d(new Vector2d(-24, -60), 0), 0)
                        .splineToSplineHeading(new Pose2d(new Vector2d(36, -60), Math.toRadians(0)), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                park(2, drive);
            }
        } else if (randomizationResult == 0) {//center
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(0, -1.5)), -Math.PI / 2), -Math.PI / 2)
                    .afterTime(0, intakeTraj(0, drive))
                    .afterTime(0, purplePixelTraj(grabyl_out, grabyr_out, drive))
                    .setReversed(false)
                    .strafeToLinearHeading(finalSpikePos.plus(new Vector2d(0, -5)), drive.pose.heading)
                    .build()
            );
            drive.updatePoseEstimate();
            if (finishEarly) {
                waitEx(1000000);
            }
            if (score) {
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(stackPosRedClose.position.plus(new Vector2d(9, -2)), 0)
                        .afterDisp(0, purplePixelTraj(grabyMid, grabyMid, drive))
                        .strafeToLinearHeading(stackPosRedClose.position.plus(new Vector2d(2, -3)), Math.toRadians(-7))
                        .build()
                );
                drive.updatePoseEstimate();
                intakeStack("frontStage", drive);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .afterDisp(5, purplePixelTraj(grabyl_out, grabyr_out, drive))
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(-40, -60, Math.toRadians(-3)), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(12, -61, Math.toRadians(-3)), Math.toRadians(0))
                        .waitSeconds(scoreDelay)
                        .splineToSplineHeading(new Pose2d(24, -52, Math.PI / 6), Math.PI / 6)
                        .splineToSplineHeading(new Pose2d(finalBackdropPos.position.plus(new Vector2d(-6, -3)), Math.toRadians(0)), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                drive.intake.setPower(0);
                scoreBackdrop(50, drive);
                waitEx(500);
                drive.intake.setPower(-1);
                waitEx(1200);
                drive.intake.setPower(0);
                scoreBackdrop(150, drive);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(finalBackdropPos.position.plus(new Vector2d(-20, 5)), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                park(2, drive);
            } else {
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(new Vector2d(-28, -60), 0), 0)
                        .splineToSplineHeading(new Pose2d(new Vector2d(36, -60), Math.toRadians(0)), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                park(2, drive);
            }
        } else {//Left
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .strafeToLinearHeading(drive.pose.position.plus(new Vector2d(0, 1)), -Math.PI / 2)
                    .splineToLinearHeading(new Pose2d(finalSpikePos.plus(new Vector2d(-11.5, -2)), -Math.PI / 2), -Math.PI / 2, slowSpeed)
                    .afterDisp(0, purplePixelTraj(grabyl_out, grabyr_out, drive))
                    .afterDisp(1, intakeTraj(0, drive))
                    .setReversed(false)
                    .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(-4, -7)), 0), 0, slowSpeed, slowAccel)
                    .build()
            );
            drive.updatePoseEstimate();
            if (finishEarly) {
                waitEx(1000000);
            }
            if (score) {
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(stackPosRedClose.position.plus(new Vector2d(9, -2)), 0)
                        .afterDisp(0, purplePixelTraj(grabyMid, grabyMid, drive))
                        .strafeToLinearHeading(stackPosRedClose.position.plus(new Vector2d(2, -3)), Math.toRadians(-7))
                        .build()
                );
                drive.updatePoseEstimate();
                intakeStack("frontStage", drive);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .afterDisp(5, purplePixelTraj(grabyl_out, grabyr_out, drive))
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(-40, -60, Math.toRadians(-3)), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(12, -61, Math.toRadians(-3)), Math.toRadians(0))
                        .waitSeconds(scoreDelay)
                        .splineToSplineHeading(new Pose2d(24, -52, Math.PI / 6), Math.PI / 6)
                        .splineToSplineHeading(new Pose2d(finalBackdropPos.position.plus(new Vector2d(-5, -0.5)), Math.toRadians(-1)), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                drive.intake.setPower(0);
                scoreBackdrop(50, drive);
                waitEx(500);
                drive.intake.setPower(-1);
                waitEx(1200);
                drive.intake.setPower(0);
                scoreBackdrop(150, drive);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(finalBackdropPos.position.plus(new Vector2d(-20, -0.5)), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                park(2, drive);

            } else {
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(false)
                        .strafeToLinearHeading(new Vector2d(-36, -60), -Math.PI / 4)
                        .strafeToLinearHeading(new Vector2d(-24, -60), 0)
                        .strafeToLinearHeading(new Vector2d(36, -58), Math.toRadians(0))
                        .build()
                );
                drive.updatePoseEstimate();
                park(2, drive);
            }
        }
    }

    private void autoPreloadedRedBackStage(@NonNull MecanumDrive drive) {
        purplePixel(false, drive);
        waitEx(500);
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .afterDisp(0, intakeTraj(0.4, drive))
                .strafeToLinearHeading(new Vector2d(finalStart.position.x, finalStart.position.y + 5), finalStart.heading)
                .afterDisp(0, intakeTraj(0.4, drive))
                .splineToSplineHeading(new Pose2d(finalBackdropPos.position.plus(new Vector2d(-7.5, -0.5)), Math.toRadians(0)), 0)
                .build()
        );
        drive.updatePoseEstimate();
        scoreBackdrop(0, drive);
        if (score && !finishEarly) {
            if (randomizationResult == -1) {//Left
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(10.5, -2)), 0), -Math.PI)
                        .build()
                );
            } else if (randomizationResult == 0) {//Center
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(0, -9)), 0), -Math.PI)
                        .build()
                );
            } else if (randomizationResult == 1) {//Right
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(-10, 0)), 0), -Math.PI)
                        .build()
                );
            }
            drive.updatePoseEstimate();
            drive.intake.setPower(0);
            purplePixel(true, drive);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(40, 10), 0)
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(finalStackPos.position.plus(new Vector2d(10, 0)), finalStackPos.heading), Math.PI)
                    .waitSeconds(0.1)
                    .splineToSplineHeading(finalStackPos, Math.PI)
                    .build()
            );
            drive.updatePoseEstimate();
            intakeStack("backStage", drive);
            waitEx(100000);
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .splineToSplineHeading(new Pose2d(-39, 10, 0), 0)
                    .splineToSplineHeading(new Pose2d(-36, 10, 0), 0)
                    .splineToSplineHeading(new Pose2d(12, 10, 0), 0)
                    .splineToSplineHeading(new Pose2d(36, 10, 0), 0)
                    .splineToSplineHeading(backdropPos5, 0)
                    .build()
            );
            drive.updatePoseEstimate();
            drive.intake.setPower(0);
            scoreBackdrop(drive);
        } else {
            if (Park() == 0) {//Far
                if (randomizationResult == -1) {//Right
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .setReversed(true)
                            .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(9.5, 4)), 0), -Math.PI)
                            .afterDisp(0, intakeTraj(0, drive))
                            .afterDisp(0, purplePixelTraj(grabyl_out, grabyr_out, drive))
                            .setReversed(false)
                            .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(-12, 0)), 0), 0)
                            .splineToSplineHeading(new Pose2d(parkFar, 0), 0)
                            .build()
                    );
                } else if (randomizationResult == 0) {//Center
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .setReversed(true)
                            .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(0, 11.5)), 0), -Math.PI)
                            .afterDisp(0, intakeTraj(0, drive))
                            .afterDisp(0, purplePixelTraj(grabyl_out, grabyr_out, drive))
                            .setReversed(false)
                            .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(-12, 0)), 0), 0)
                            .splineToSplineHeading(new Pose2d(parkFar, 0), 0)
                            .build()
                    );
                } else if (randomizationResult == 1) {//Left
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .setReversed(true)
                            .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(-12, 4)), 0), -Math.PI)
                            .afterDisp(0, intakeTraj(0, drive))
                            .afterDisp(0, purplePixelTraj(grabyl_out, grabyr_out, drive))
                            .setReversed(false)
                            .strafeToLinearHeading(finalSpikePos.plus(new Vector2d(-5, -4)), 0)
                            .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(0,-1)), 0), 0)
                            .build()
                    );
                }
            } else {//Close
                if (randomizationResult == -1) {//Right
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .setReversed(true)
                            .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(10.0, 4)), 0), -Math.PI)
                            .afterDisp(0, intakeTraj(0, drive))
                            .afterDisp(0, purplePixelTraj(grabyl_out, grabyr_out, drive))
                            .setReversed(false)
                            .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(-12, 0)), 0), 0)
                            .splineToSplineHeading(new Pose2d(parkClose, 0), 0)
                            .build()
                    );
                } else if (randomizationResult == 0) {//Center
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .setReversed(true)
                            .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(0, 11.5)), 0), -Math.PI)
                            .afterDisp(0, intakeTraj(0, drive))
                            .afterDisp(0, purplePixelTraj(grabyl_out, grabyr_out, drive))
                            .setReversed(false)
                            .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(-12, 0)), 0), 0)
                            .splineToSplineHeading(new Pose2d(parkClose, 0), 0)
                            .build()
                    );
                } else if (randomizationResult == 1) {//Left
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .setReversed(true)
                            .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(-12, 4)), 0), -Math.PI)
                            .afterDisp(0, intakeTraj(0, drive))
                            .afterDisp(0, purplePixelTraj(grabyl_out, grabyr_out, drive))
                            .setReversed(false)
                            .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(-12, 0)), 0), 0)
                            .splineToSplineHeading(new Pose2d(parkClose, 0), 0)
                            .build()
                    );
                }
            }
        }
    }

    /**
     * 0==blueFront
     * 1==blueBack
     * 2==redFront
     * 3==redBack
     */
    public void park(int location, @NonNull MecanumDrive drive) {
        switch (location) {
            case 0:
                if (Park() == 0) {//Far
                    if (Side() == 0 && randomizationResult == -1) {//FrontStage left
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(-12, 0)), 0), 0)
                                .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(0, 0)), 0), 0)
                                .build()
                        );
                    } else if (Side() == 0 && randomizationResult == 0) {//center
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(-12, -4)), 0), 0)
                                .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(0, -4)), 0), 0)
                                .build()
                        );
                    } else if (Side() == 0 && randomizationResult == 1) {//right
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(-12, -7)), 0), 0)
                                .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(0, -7)), 0), 0)
                                .build()
                        );
                    } else {//Backstage
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(-12, 1)), 0), 0)
                                .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(0, 1)), 0), 0)
                                .build()
                        );
                    }
                } else if (Park() == 1) {//Close
                    if (Side() == 0 && randomizationResult == -1) {//FrontStage left
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(-12, -12)), 0), 0)
                                .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(3, -12)), 0), 0)
                                .build()
                        );
                    } else if (Side() == 0 && randomizationResult == 0) {//center
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(-12, -5)), 0), 0)
                                .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(3, -5)), 0), 0)
                                .build()
                        );
                    } else {//Backstage
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(-12, -5)), 0), 0)
                                .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(0, -5)), 0), 0)
                                .build()
                        );
                    }
                }
                break;
            case 1:
                if (Park() == 1) {//Close
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(-12, 0)), 0), 0)
                            .splineToSplineHeading(new Pose2d(parkClose, 0), 0)
                            .build()
                    );
                }
                break;
            case 2:
                if (Park() == 0) {//Far
                    if (Side() == 0 && randomizationResult == -1) {//FrontStage Right
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(-12, -16)), 0), 0)
                                .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(0, -16)), 0), 0)
                                .build()
                        );
                    } else if (Side() == 0 && randomizationResult == 0) {//Center
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(-12, -4)), 0), 0)
                                .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(0, -4)), 0), 0)
                                .build()
                        );
                    } else if (Side() == 0 && randomizationResult == 1) {//Left
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(-12, -3)), 0), 0)
                                .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(0, -3)), 0), 0)
                                .build()
                        );
                    } else {//Backstage
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(-12, 1)), 0), 0)
                                .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(0, 1)), 0), 0)
                                .build()
                        );
                    }
                } else if (Park() == 1) {//Close
                    if (Side() == 0 && randomizationResult == -1) {//FrontStage right
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(-12, 12)), 0), 0)
                                .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(3, 12)), 0), 0)
                                .build()
                        );
                    } else if (Side() == 0 && randomizationResult == 0) {//center
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(-12, 5)), 0), 0)
                                .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(3, 5)), 0), 0)
                                .build()
                        );
                    } else {//Backstage
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(-12, 5)), 0), 0)
                                .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(0, 5)), 0), 0)
                                .build()
                        );
                    }
                }
                break;
            case 3:
                if (Park() == 0) {//Far
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(-12, -1)), 0), 0)
                            .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(0, -1)), 0), 0)
                            .build()
                    );
                } else if (Park() == 1) {//Close
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(-12, -2)), 0), 0)
                            .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(0, -2)), 0), 0)
                            .build()
                    );
                }
            default:
                break;
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
        lifts(225, drive);
        waitEx(500);
        drive.flipper.setPosition(flipperScore);
        waitEx(1850);
        drive.flipper.setPosition(flipperIntake);
        waitEx(150);
        lifts(0, drive);
        waitEx(50);
    }

    private void scoreBackdrop(int extraHeight, @NonNull MecanumDrive drive) {
        waitEx(500);
        lifts(225 + extraHeight, drive);
        waitEx(500);
        drive.flipper.setPosition(flipperScore);
        waitEx(1850);
        drive.flipper.setPosition(flipperIntake);
        waitEx(150);
        lifts(0, drive);
        waitEx(50);
    }

    private void intakeStack(String type, @NonNull MecanumDrive drive) {
        switch (type) {
            case "frontStage":
                drive.flipper.setPosition(0.9);
                lifts(0, drive);
                drive.intake.setPower(-1);
                drive.grabyL.setPosition(grabyl_in);
                drive.grabyR.setPosition(grabyr_in);
                waitEx(550);
                drive.intake.setPower(0);
                break;
            case "backStage":
            default:
                drive.intake.setPower(-1);
                drive.grabyL.setPosition(grabyl_in);
                drive.grabyR.setPosition(grabyr_in);
                waitEx(300);
                drive.grabyL.setPosition(grabyl_out);
                drive.grabyR.setPosition(grabyr_out);
                waitEx(300);
                drive.grabyL.setPosition(grabyl_in);
                drive.grabyR.setPosition(grabyr_in);
                waitEx(300);
                drive.grabyL.setPosition(grabyl_out);
                drive.grabyR.setPosition(grabyr_out);
                waitEx(300);
                drive.intake.setPower(-1);
                break;
        }
    }

    private void lifts(int pos, @NonNull MecanumDrive drive) {
        drive.leftLift.setTargetPosition(pos);
        drive.rightLift.setTargetPosition(pos);
        drive.leftLift.setPower(0.5);
        drive.rightLift.setPower(0.5);
    }

    private void purplePixel(boolean open, @NonNull MecanumDrive drive) {
        if (open) {
            drive.intake.setPower(0.25);
            drive.grabyL.setPosition(0.55);
            drive.grabyR.setPosition(0.75);
            waitEx(100);
            drive.intake.setPower(0);
            waitEx(100);
            drive.grabyL.setPosition(grabyl_out);
            drive.grabyR.setPosition(grabyr_out);

        } else {
            drive.grabyL.setPosition(grabyl_in);
            drive.grabyR.setPosition(grabyr_in);
        }
    }

    private void purplePixel(double position, @NonNull MecanumDrive drive) {
        drive.grabyL.setPosition(1 - position);
        drive.grabyR.setPosition(position);
    }

    private void purplePixel(double positionL, double positionR, @NonNull MecanumDrive drive) {
        drive.grabyL.setPosition(positionL);
        drive.grabyR.setPosition(positionR);
    }

    public Action purplePixelTraj(double positionL, double positionR, @NonNull MecanumDrive drive) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    drive.grabyL.setPosition(positionL);
                    drive.grabyR.setPosition(positionR);
                    initialized = true;
                }
                return false;
            }
        };
    }

    public Action intakeTraj(double power, @NonNull MecanumDrive drive) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    drive.intake.setPower(power);
                    initialized = true;
                }
                return false;
            }
        };
    }
}
