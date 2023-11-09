package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config

public class DriveCodeCommon extends LinearOpMode {
    public double latchClosed = 0;
    public double latchOpen1 = 0.5;
    public double latchOpen2 = 1;


    //    public static double[] latch = {latchClosed,latchOpen1,latchOpen2};
    double[] latch = {0.74, 0.79, 0.9};
    public double flipperscore = 0.55;
    public double flipperintake = 1;

    double liftModSum = 0;
    int finalLiftPos = 0;

    GamepadEx x1 = new GamepadEx();
    GamepadEx y1 = new GamepadEx();
    GamepadEx lb1 = new GamepadEx();
    GamepadEx a1 = new GamepadEx(4, true);
    GamepadEx b1 = new GamepadEx(4, false);
    GamepadEx a2 = new GamepadEx();
    GamepadEx b2 = new GamepadEx();
    GamepadEx dpadL1 = new GamepadEx(5,true);
    int backSpeed = -200;
    int forwardSpeed = 1650;
    double planeClosed = 0.35;
    double planeOpen = 0.6;
    int planeTargetPos = 0;
    ElapsedTime planeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    int[] liftTargetPos = {0, 100, 200, 300};

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void Initialization(MecanumDrive drive) {
        telemetry.update();
        drive.leftLift.setTargetPosition(liftTargetPos[0]);
        drive.rightLift.setTargetPosition(liftTargetPos[0]);
        drive.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.planeLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.planeLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
    }

    public void updateButtons(MecanumDrive drive) {
        x1.updateButton(gamepad1.x);
        y1.updateButton(gamepad1.y);
        lb1.updateButton(gamepad1.left_bumper);
        a1.updateButton(gamepad1.a);
        if (a1.isPressed()) {
            b1.setToggle(a1.getCycle());
        }
        b1.updateButton(gamepad1.b);
        if (b1.isPressed()) {
            a1.setToggle(b1.getCycle());
        }
        a2.updateButton(gamepad2.a);
        b2.updateButton(gamepad2.b);
    }

    public void rawDriving(MecanumDrive drive) {
        drive.frontLeft.setPower (-gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.right_stick_x * 1));
        drive.frontRight.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.right_stick_x * 1));
        drive.backRight.setPower (-gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.right_stick_x * 1));
        drive.backLeft.setPower  (-gamepad1.left_stick_y - gamepad1.left_stick_x + (gamepad1.right_stick_x * 1));

    }

    public void intake(MecanumDrive drive) {
        if (x1.isToggled()) {
            drive.intake.setPower(-1);
        } else if (lb1.isPressed()) {
            drive.intake.setPower(1);
        } else {
            drive.intake.setPower(0);
        }
    }

    public void lift(MecanumDrive drive) {
        drive.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.leftLift.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
        drive.rightLift.setPower(gamepad1.right_trigger-gamepad1.left_trigger);

//        liftModSum += gamepad1.right_trigger - gamepad1.left_trigger;
//        if (gamepad1.right_trigger + gamepad1.left_trigger <= 0.01) {
//            liftModSum = 0;
//        }
//        finalLiftPos = Range.clip(liftTargetPos[a1.getCycle()] + Math.toIntExact(Math.round(liftModSum)), 0, 20000);
//        drive.leftLift.setPower(1);
//        drive.rightLift.setPower(1);
//        drive.leftLift.setTargetPosition(finalLiftPos);
//        drive.rightLift.setTargetPosition(finalLiftPos);
    }

    public void outake(MecanumDrive drive) {
        if (y1.isToggled()) {
            drive.flipper.setPosition(flipperscore);
        } else {
            drive.flipper.setPosition(flipperintake);
        }
    }

    public void launcher(MecanumDrive drive) {
        switch (dpadL1.getCycle()) {
            case 0:
                drive.launchLatch.setPosition(planeClosed);
                drive.planeLauncher.setTargetPosition(planeTargetPos);
                drive.planeLauncher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.planeLauncher.setPower(1);
                dpadL1.updateButton(gamepad1.dpad_left);
                planeTimer.reset();
                break;
            case 1:
                drive.planeLauncher.setPower(0);
                drive.planeLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (planeTimer.milliseconds() > 500) {
                    planeTimer.reset();
                    dpadL1.setToggle(dpadL1.getCycle() + 1);
                }
                break;
            case 2:
                drive.planeLauncher.setVelocity(backSpeed);
                if (planeTimer.milliseconds() > 1000) {
                    planeTimer.reset();
                    dpadL1.setToggle(dpadL1.getCycle() + 1);
                }
                break;
            case 3:
                drive.planeLauncher.setVelocity(forwardSpeed);
                if (Math.abs(drive.planeLauncher.getVelocity() - forwardSpeed) < 20) {
                    planeTimer.reset();
                    dpadL1.setToggle(dpadL1.getCycle() + 1);
                }
                break;
            case 4:
                drive.launchLatch.setPosition(planeOpen);
                planeTargetPos = drive.planeLauncher.getCurrentPosition();
                if (planeTimer.milliseconds() > 500) {
                    dpadL1.setToggle(0);
                }
                break;
        }
    }

    public void telemetry(MecanumDrive drive) {
        telemetry.addData("Launcher Velocity: ", drive.planeLauncher.getVelocity());
        telemetry.addData("b2.isToggled: ", b2.isToggled());
        telemetry.addData("Servo Pos: ", drive.launchLatch.getPosition());
        telemetry.addData("Launcher Power; ", drive.planeLauncher.getPower());
        telemetry.addData("Lift encoder Left", drive.leftLift.getCurrentPosition());
        telemetry.addData("Lift encoder Right", drive.rightLift.getCurrentPosition());
        telemetry.update();
        dashboardTelemetry.addData("Launcher Velocity: ", drive.planeLauncher.getVelocity());
        dashboardTelemetry.addData("Servo Pos: ", drive.launchLatch.getPosition());
        dashboardTelemetry.addData("b2.isToggled: ", b2.isToggled());
        dashboardTelemetry.addData("Launcher Power; ", drive.planeLauncher.getPower());
        dashboardTelemetry.update();
    }

}
