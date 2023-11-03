        package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.hardware.PIDFCoefficients;
        import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class HardwareMapTeam extends LinearOpMode {
    public final DcMotorEx frontLeft, backLeft, backRight, frontRight, leftLift, rightLift, intake, planeLauncher;
    public final Servo outakeLatch,flipper,launchLatch;
    public HardwareMapTeam(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx .class, "fl");
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");

        leftLift = hardwareMap.get(DcMotorEx.class,"ll");
        rightLift = hardwareMap.get(DcMotorEx.class,"rl");
        intake = hardwareMap.get(DcMotorEx.class, "i");
        planeLauncher = hardwareMap.get(DcMotorEx.class,"pl");
        outakeLatch = hardwareMap.get(Servo.class,"o");
        flipper = hardwareMap.get(Servo.class,"f");
        launchLatch = hardwareMap.get(Servo.class,"l");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
        planeLauncher.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

//        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
//        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
//        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
//        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
//
//        frontLeft .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        frontLeft .setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeft  .setDirection(DcMotorSimple.Direction.REVERSE);
//        backRight .setDirection(DcMotorSimple.Direction.FORWARD);
//        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        while(!isStarted()&&!isStopRequested()) {

        }
    }
}