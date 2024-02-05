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
    final Pose2d backdropPos1 = new Pose2d(59.5, 42, 0);
    final Pose2d backdropPos2 = new Pose2d(59.5, 35, 0);
    final Pose2d backdropPos3 = new Pose2d(59.5, 27, 0);
    final Pose2d backdropPos4 = new Pose2d(59.5, -29, 0);
    final Pose2d backdropPos5 = new Pose2d(59.5, -36, 0);
    final Pose2d backdropPos6 = new Pose2d(59.5, -42.75, 0);
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
            yrPlus.updateButton(-gamepad1.right_stick_y > 0);
            yrMinus.updateButton(-gamepad1.right_stick_y < 0);
            ylPlus.updateButton(-gamepad1.left_stick_y > 0);
            ylMinus.updateButton(-gamepad1.left_stick_y < 0);

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
            } else {
                telemetry.addData("Side", "BackStage");
            }
            if (Park() == 0) {
                telemetry.addData("Park", "Far");
            } else {
                telemetry.addData("Park", "Close");
            }
            if (Side() == 0) {
                if (Path() == 0) {
                    telemetry.addData("Path", "Rigging");
                } else {
                    telemetry.addData("Path", "Stage Door");
                }
                telemetry.addData("Finish After Purple", finishEarly);
                if (!finishEarly) {
                    telemetry.addData("Scoring?", score);
                }
                telemetry.addData("Start Delay (seconds)", startDelay);
                telemetry.addData("Score delay (seconds)", scoreDelay);
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
                if (l1.isPressed()) {
                    path++;
                }
                if (r1.isPressed()) {

                }
                if (u1.isPressed() && Side() == 0 && !finishEarly) {
                    score = !score;
                }
                if (d1.isPressed() && Side() == 0) {
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

    //TODO TEST
    private void autoPreloadedBlueFrontStage(@NonNull MecanumDrive drive) {
        drive.ppp.setPosition(ppp_down);
        if (randomizationResult == -1) {//left
            if (Path() == 0) {//rigging
                drive.ppp.setPosition(ppp_down);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(4, -4)), Math.PI), 0)
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
                if (score) {
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .setReversed(false)
                            .strafeToLinearHeading(new Vector2d(-48, finalSpikePos.y), Math.toRadians(90))
                            .waitSeconds(0.125)
                            .strafeToLinearHeading(new Vector2d(-40, 55), Math.toRadians(0))
//                            .splineToSplineHeading(new Pose2d(-48, 48, Math.PI / 2), Math.PI / 2)
//                            .strafeToLinearHeading(new Vector2d(-29, 64), Math.toRadians(2))
                            .splineToSplineHeading(new Pose2d(new Vector2d(12, 60), Math.toRadians(0)), Math.toRadians(0))
                            .build()
                    );
                    drive.updatePoseEstimate();
                    telemetry.addData("Pose",drive.pose);
                    telemetry.update();
                    waitEx(scoreDelay * 1000);
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .splineToSplineHeading(new Pose2d(24, 52, -Math.PI / 6), -Math.PI / 6)
                            .splineToSplineHeading(new Pose2d(finalBackdropPos.position.plus(new Vector2d(-6, -11.5)), Math.toRadians(-5)), 0)
                            .build()
                    );
                    drive.updatePoseEstimate();
                    scoreBackdrop(drive);
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(finalBackdropPos.position.plus(new Vector2d(-20, -11.5)), 0)
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
            } else {//stage door

            }
        } else if (randomizationResult == 0) {//center
            if (Path() == 0) {//rigging
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(0, 2)), Math.PI / 2), Math.PI / 2)
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
                if (score) {
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .setReversed(false)
                            .splineToSplineHeading(new Pose2d(new Vector2d(-28, 60), 0), 0)
                            .splineToSplineHeading(new Pose2d(new Vector2d(12, 60), Math.toRadians(0)), 0)
                            .build()
                    );
                    drive.updatePoseEstimate();
                    waitEx(scoreDelay * 1000);
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .splineToSplineHeading(new Pose2d(24, 52, -Math.PI / 6), -Math.PI / 6)
                            .splineToSplineHeading(new Pose2d(finalBackdropPos.position.plus(new Vector2d(-6, -5)), Math.toRadians(-5)), 0)
                            .build()
                    );
                    drive.updatePoseEstimate();
                    scoreBackdrop(drive);
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
                        .strafeToLinearHeading(new Vector2d(12, 12), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                park(0, drive);
            }
        } else {//Right
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
                if (score) {
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .setReversed(false)
                            .strafeToLinearHeading(new Vector2d(-36, 60), Math.PI / 4)
                            .strafeToLinearHeading(new Vector2d(-24, 60), 0)
                            .strafeToLinearHeading(new Vector2d(12, 58), Math.toRadians(0))
                            .build()
                    );
                    drive.updatePoseEstimate();
                    waitEx(scoreDelay * 1000);
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .splineToSplineHeading(new Pose2d(24, 52, -Math.PI / 6), -Math.PI / 6)
                            .splineToSplineHeading(new Pose2d(finalBackdropPos.position.plus(new Vector2d(-8, -3)), 0), 0)
                            .build()
                    );
                    drive.updatePoseEstimate();
                    scoreBackdrop(drive);
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(finalBackdropPos.position.plus(new Vector2d(-20, -3)), 0)
                            .build()
                    );
                    drive.updatePoseEstimate();
                    park(2, drive);
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
        if (randomizationResult == -1) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(-10 * randomizationResult, yMod * 11.5 * ((randomizationResult + 1) % 2))), headingMod), headingMod - Math.PI)
                    .build()
            );
        } else if (randomizationResult == 0) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(-9.5 * randomizationResult, yMod * 11.5 * ((randomizationResult + 1) % 2))), headingMod), headingMod - Math.PI)
                    .build()
            );
        } else if (randomizationResult == 1) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(-9.5 * randomizationResult, yMod * 11.5 * ((randomizationResult + 1) % 2))), headingMod), headingMod - Math.PI)
                    .build()
            );
        }
        drive.ppp.setPosition(ppp_up);
        park(1, drive);
    }

    private void autoPreloadedRedFrontStage(@NonNull MecanumDrive drive) {
        drive.ppp.setPosition(ppp_down);
        if (randomizationResult == -1) {//right
            if (Path() == 0) {//rigging
                drive.ppp.setPosition(ppp_down);
                Actions.runBlocking(drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(3, 4)), Math.PI), 0)
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
                if (score) {
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .setReversed(false)
                            .splineToSplineHeading(new Pose2d(-48, -48, -Math.PI / 2), -Math.PI / 2)
                            .splineToSplineHeading(new Pose2d(new Vector2d(-24, -62), 0), 0)
                            .splineToSplineHeading(new Pose2d(new Vector2d(12, -62), Math.toRadians(0)), 0)
                            .build()
                    );
                    drive.updatePoseEstimate();
                    waitEx(scoreDelay * 1000);
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .splineToSplineHeading(new Pose2d(24, -52, Math.PI / 6), Math.PI / 6)
                            .splineToSplineHeading(new Pose2d(finalBackdropPos.position.plus(new Vector2d(-3, -6.75)), 0), 0)
                            .build()
                    );
                    drive.updatePoseEstimate();
                    scoreBackdrop(drive);
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(finalBackdropPos.position.plus(new Vector2d(-18, -6.75)), 0)
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
                if (score) {
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .setReversed(false)
                            .splineToSplineHeading(new Pose2d(new Vector2d(-28, -60), 0), 0)
                            .splineToSplineHeading(new Pose2d(new Vector2d(12, -60), Math.toRadians(0)), 0)
                            .build()
                    );
                    drive.updatePoseEstimate();
                    waitEx(scoreDelay * 1000);
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .splineToSplineHeading(new Pose2d(24, -52, Math.PI / 6), Math.PI / 6)
                            .splineToSplineHeading(new Pose2d(finalBackdropPos.position.plus(new Vector2d(-4.5, -4)), 0), 0)
                            .build()
                    );
                    drive.updatePoseEstimate();
                    scoreBackdrop(drive);
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(finalBackdropPos.position.plus(new Vector2d(-18.5, -4)), 0)
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
                        .strafeToLinearHeading(new Vector2d(12, -12), 0)
                        .build()
                );
                drive.updatePoseEstimate();
                park(2, drive);
            }
        } else {//left
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
                if (score) {
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .setReversed(false)
                            .strafeToLinearHeading(new Vector2d(-36, -60), -Math.PI / 4)
                            .strafeToLinearHeading(new Vector2d(-24, -61), 0)
                            .strafeToLinearHeading(new Vector2d(12, -61), Math.toRadians(0))
                            .build()
                    );
                    drive.updatePoseEstimate();
                    waitEx(scoreDelay * 1000);
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .splineToSplineHeading(new Pose2d(24, -52, Math.PI / 6), Math.PI / 6)
                            .splineToSplineHeading(new Pose2d(finalBackdropPos.position.plus(new Vector2d(-5, -3)), 0), 0)
                            .build()
                    );
                    drive.updatePoseEstimate();
                    scoreBackdrop(drive);
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(finalBackdropPos.position.plus(new Vector2d(-20, -3)), 0)
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
        if (randomizationResult == -1) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(10.5, 0)), headingMod), headingMod - Math.PI)
                    .build()
            );
        } else if (randomizationResult == 0) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(-13 * randomizationResult, yMod * 9.5 * ((randomizationResult + 1) % 2))), headingMod), headingMod - Math.PI)
                    .build()
            );
        } else if (randomizationResult == 1) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(finalSpikePos.plus(new Vector2d(-12.5 * randomizationResult, yMod * 9.5 * ((randomizationResult + 1) % 2))), headingMod), headingMod - Math.PI)
                    .build()
            );
        }
        drive.updatePoseEstimate();
        drive.ppp.setPosition(ppp_up);
        park(3, drive);
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
                    Actions.runBlocking(drive.actionBuilder(drive.pose)
                            .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(-12, 0)), 0), 0)
                            .splineToSplineHeading(new Pose2d(parkFar, 0), 0)
                            .build()
                    );
                } else if (Park() == 1) {//Close
                    if(Side()==0 && randomizationResult==-1) {//FrontStage left
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(-12, -12)), 0), 0)
                                .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(3,-12)), 0), 0)
                                .build()
                        );
                    } else if (Side()==0 && randomizationResult==0) {//center
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(-12, -5)), 0), 0)
                                .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(3,-5)), 0), 0)
                                .build()
                        );
                    } else {//Backstage
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(-12, -5)), 0), 0)
                                .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(0,-5)), 0), 0)
                                .build()
                        );
                    }
                }
                break;
            case 1:
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
                break;
            case 2:
                if (Side() == 0) {//Frontstage
                    if (Park() == 0) {//Far
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(-12, -6)), Math.toRadians(-5)), 0)
                                .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(0, -6)), Math.toRadians(-5)), 0)
                                .build()
                        );
                    } else if (Park() == 1) {//Close
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(-12, 0)), 0), 0)
                                .splineToSplineHeading(new Pose2d(parkClose, 0), 0)
                                .build()
                        );
                    }
                } else {
                    if (Park() == 0) {//Far
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(-12, -2)), Math.toRadians(-5)), 0)
                                .splineToSplineHeading(new Pose2d(parkFar.plus(new Vector2d(0, -2)), Math.toRadians(-5)), 0)
                                .build()
                        );
                    } else if (Park() == 1) {//Close
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineToSplineHeading(new Pose2d(parkClose.plus(new Vector2d(-12, 0)), 0), 0)
                                .splineToSplineHeading(new Pose2d(parkClose, 0), 0)
                                .build()
                        );
                    }
                }//Backstage

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

    private void lifts(int pos, @NonNull MecanumDrive drive) {
        drive.leftLift.setTargetPosition(pos);
        drive.rightLift.setTargetPosition(pos);
        drive.leftLift.setPower(0.5);
        drive.rightLift.setPower(0.5);
    }


}
