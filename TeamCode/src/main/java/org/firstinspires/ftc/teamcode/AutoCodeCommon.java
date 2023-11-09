package org.firstinspires.ftc.teamcode;

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

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void setup(MecanumDrive drive) {
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

    public void initialization(MecanumDrive drive) {
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
                positionAnalysis(drive);
            } else {
                if (b1.isPressed()) {
                    team++;
                }

                if (x1.isPressed()) {
                    side++;
                }
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
        if(Team() == 0) {
            if(pipelineBlue.getAnalysis() == ColorDetection.BlueDeterminationPipeline.TeamElementPosition.LEFT) {
                randomizationResult = 0;
            } else if(pipelineBlue.getAnalysis() == ColorDetection.BlueDeterminationPipeline.TeamElementPosition.CENTER) {
                randomizationResult = 1;
            } else if(pipelineBlue.getAnalysis() == ColorDetection.BlueDeterminationPipeline.TeamElementPosition.RIGHT) {
                randomizationResult = 2;
            }
        } else {
            if(pipelineRed.getAnalysis() == ColorDetection.RedDeterminationPipeline.TeamElementPosition.LEFT) {
                randomizationResult = 0;
            } else if(pipelineRed.getAnalysis() == ColorDetection.RedDeterminationPipeline.TeamElementPosition.CENTER) {
                randomizationResult = 1;
            } else if(pipelineRed.getAnalysis() == ColorDetection.RedDeterminationPipeline.TeamElementPosition.RIGHT) {
                randomizationResult = 2;
            }
        }
    }

    public void positionAnalysis(MecanumDrive drive) {

    }

    public void buildTrajectories(MecanumDrive drive) {

    }

    public void scorePreloadedFloor(MecanumDrive drive) {

    }

    public void driveToBackStage(MecanumDrive drive) {

    }

    public void scorePreloadedBackdrop(MecanumDrive drive) {

    }

    public void park(MecanumDrive drive) {

    }

}
