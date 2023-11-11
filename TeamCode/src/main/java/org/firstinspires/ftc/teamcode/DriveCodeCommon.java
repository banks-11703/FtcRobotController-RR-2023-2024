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
    double driveSpeedMod = 1;
    double DriveSpeedMod() {
        if(gamepad1.left_trigger+gamepad1.right_trigger>0.1) {
            return 0.5;
        } else {
            return 1;
        }
    }

    GamepadEx x1 = new GamepadEx();//intake
    GamepadEx b1 = new GamepadEx();//intake out
    GamepadEx x2 = new GamepadEx();//outtake
    GamepadEx a2 = new GamepadEx();//liftToZero
    GamepadEx b2 = new GamepadEx(4, false);//liftdown
    GamepadEx dpadL2 = new GamepadEx(5, true);//planeLauncher
    GamepadEx y2 = new GamepadEx();

    int backSpeed = -200;
    int forwardSpeed = 1650;
    double planeClosed = 0.35;
    double planeOpen = 0.6;
    int planeTargetPos = 0;
    ElapsedTime planeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    boolean liftBusy = false;
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
        x2.updateButton(gamepad2.x);
        b1.updateButton(gamepad1.b);
        a2.updateButton(gamepad2.a);
        b2.updateButton(gamepad2.b);
        y2.updateButton(gamepad2.y);
    }

    public void rawDriving(MecanumDrive drive) {
        drive.frontLeft.setPower ((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)*DriveSpeedMod());
        drive.frontRight.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)*DriveSpeedMod());
        drive.backRight.setPower ((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x)*DriveSpeedMod());
        drive.backLeft.setPower  ((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x)*DriveSpeedMod());

    }

    public void intake(MecanumDrive drive) {//1
        if (x1.isToggled()) {
            drive.intake.setPower(-1);
        } else if (b1.isHeld()) {
            drive.intake.setPower(1);
        } else {
            drive.intake.setPower(0);
        }
    }

    public void lift(MecanumDrive drive) {//2
        if(a2.isPressed()) {
            liftBusy = true;
            drive.leftLift.setTargetPosition(0);
            drive.rightLift.setTargetPosition(0);
            drive.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.leftLift.setPower(1);
            drive.leftLift.setPower(1);
        }
        if(liftBusy) {
            if(Math.abs(drive.leftLift.getTargetPosition()-drive.leftLift.getCurrentPosition())+Math.abs(drive.rightLift.getTargetPosition()-drive.rightLift.getCurrentPosition())<= 50) {
                liftBusy = false;
            }
        } else {
            drive.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (drive.leftLift.getCurrentPosition() < 1930) {
                drive.leftLift.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
                drive.rightLift.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
            } else {
                drive.leftLift.setPower(-gamepad2.left_trigger);
                drive.rightLift.setPower(-gamepad2.left_trigger);
            }
        }



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

    public void outake(MecanumDrive drive) {//2
        if (x2.isHeld()) {
            drive.flipper.setPosition(flipperscore);
        } else {
            drive.flipper.setPosition(flipperintake);
        }
    }

    public void launcher(MecanumDrive drive) {//2
        switch (dpadL2.getCycle()) {
            case 0:
                drive.launchLatch.setPosition(planeClosed);
                drive.planeLauncher.setTargetPosition(planeTargetPos);
                drive.planeLauncher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.planeLauncher.setPower(1);
                dpadL2.updateButton(gamepad2.dpad_left);
                planeTimer.reset();
                break;
            case 1:
                drive.planeLauncher.setPower(0);
                drive.planeLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (planeTimer.milliseconds() > 500) {
                    planeTimer.reset();
                    dpadL2.setToggle(dpadL2.getCycle() + 1);
                }
                break;
            case 2:
                drive.planeLauncher.setVelocity(backSpeed);
                if (planeTimer.milliseconds() > 1000) {
                    planeTimer.reset();
                    dpadL2.setToggle(dpadL2.getCycle() + 1);
                }
                break;
            case 3:
                drive.planeLauncher.setVelocity(forwardSpeed);
                if (Math.abs(drive.planeLauncher.getVelocity() - forwardSpeed) < 20) {
                    planeTimer.reset();
                    dpadL2.setToggle(dpadL2.getCycle() + 1);
                }
                break;
            case 4:
                drive.launchLatch.setPosition(planeOpen);
                planeTargetPos = drive.planeLauncher.getCurrentPosition();
                if (planeTimer.milliseconds() > 500) {
                    dpadL2.setToggle(0);
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
