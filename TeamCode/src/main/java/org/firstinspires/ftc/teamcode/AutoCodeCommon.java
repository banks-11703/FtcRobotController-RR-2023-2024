package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
@Disabled
public class AutoCodeCommon extends LinearOpMode {

    OpenCvCamera camera;
    public ColorDetection.RedDeterminationPipeline pipelineRed;

    public ColorDetection.BlueDeterminationPipeline pipelineBlue;

    boolean lockedIn = false;
    boolean wasLockedIn = false;
    int team = 0;

    int Team() {
        return team % 2;
    }//0==Blue   1==Red

    int side = 0;

    int Side() {
        return side % 2;
    }  //0==Left   1==Right

    GamepadEx a1 = new GamepadEx();
    GamepadEx b1 = new GamepadEx();
    GamepadEx x1 = new GamepadEx();

    int randomizationResult;//0==left   1==center   2==right


    int yMod = 1;
    int xMod = 0;
    Pose2d startRedLeft = new Pose2d(-36, -60, Math.toRadians(90));
    Pose2d startRedRight = new Pose2d(36, -60, Math.toRadians(90));
    Pose2d startBlueLeft = new Pose2d(36, 60, Math.toRadians(-90));
    Pose2d startBlueRight = new Pose2d(-36, 60, Math.toRadians(-90));
    Pose2d finalStart;

    Pose2d parkLower;
    Pose2d parkUpper;
    Pose2d scoreClose;
    Pose2d scoreMiddle;
    Pose2d scoreFar;
    Pose2d spikeLeft;
    Pose2d spikeCenter;
    Pose2d spikeRight;


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
            if (lockedIn) {
                if (Team() == 0) {
                    camera.setPipeline(pipelineBlue);
                    telemetry.addData("Analysis", pipelineBlue.getAnalysis());
                    telemetry.addData("Team: ", "Blue");
                } else {
                    camera.setPipeline(pipelineRed);
                    telemetry.addData("Analysis", pipelineRed.getAnalysis());
                    telemetry.addData("Team: ", "Red");
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

            if (lockedIn) {
                telemetry.addData("Press A to back out", "");
            } else {
                telemetry.addData("Press A to lock in decision", "");
            }
            if (Team() == 0) {
                telemetry.addData("Team: ", "Blue");
            } else {
                telemetry.addData("Team: ", "Red");
            }
            if (Side() == 0) {
                telemetry.addData("Side: ", "Left");
            } else {
                telemetry.addData("Side: ", "Right");
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
        if (Team() == 0 && Side() == 0) {//blue left
            yMod = -1;
            xMod = 72;
            finalStart = startBlueLeft;
        } else if (Team() == 0 && Side() == 1) {//blue right
            yMod = -1;
            xMod = 0;
            finalStart = startBlueRight;
        } else if (Team() == 1 && Side() == 0) {//red left
            yMod = 1;
            xMod = 0;
            finalStart = startRedLeft;
        } else if (Team() == 1 && Side() == 1) {//red right
            yMod = 1;
            xMod = 72;
            finalStart = startRedRight;
        }

        //all positions with y,x,or heading Mod assume red left starting
        parkLower   = new Pose2d(-36 + xMod, -60 * yMod, Math.toRadians(90 * yMod));
        parkUpper   = new Pose2d(-36 + xMod, -60 * yMod, Math.toRadians(90 * yMod));
        scoreClose  = new Pose2d(-36 + xMod, -60 * yMod, Math.toRadians(90 * yMod));
        scoreMiddle = new Pose2d(-36 + xMod, -60 * yMod, Math.toRadians(90 * yMod));
        scoreFar    = new Pose2d(-36 + xMod, -60 * yMod, Math.toRadians(90 * yMod));
        spikeLeft   = new Pose2d(-36 + xMod, -60 * yMod, Math.toRadians(90 * yMod));
        spikeCenter = new Pose2d(-36 + xMod, -60 * yMod, Math.toRadians(90 * yMod));
        spikeRight  = new Pose2d(-36 + xMod, -60 * yMod, Math.toRadians(90 * yMod));
    }

    public void buildTrajectories(MecanumDrive drive) {
//        Action a = drive.actionBuilder(drive.pose)
//                .strafeTo(new Vector2d(0,10))
//                .build();
    }

    public void scorePreloadedFloor(MecanumDrive drive) {
//        Actions.runBlocking(a);
    }

    public void driveToBackStage(MecanumDrive drive) {

    }

    public void scorePreloadedBackdrop(MecanumDrive drive) {

    }

    public void park(MecanumDrive drive) {

    }

}
