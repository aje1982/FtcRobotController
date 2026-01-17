package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Drive")
public class Drive extends LinearOpMode {

    private DcMotor intake;
    private DcMotor intake2;
    private DcMotorEx outtake;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private Servo gate;
    private Servo hood;
    private double targetVelocity = 0;

    double intakePower;
    double hoodPosition;
    int nearVelocity;
    int farVelocity;
    double gateClosePos;
    String teleOp2;
    String autoBlue2;
    String autoRed2;
    double gateOpenPos;


    @Override
    public void runOpMode() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        gate = hardwareMap.get(Servo.class, "gate");
        hood = hardwareMap.get(Servo.class, "hood");
        // Put initialization blocks here.
        initRobot();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                mecanumDrive();
                manualHoodControl();
                manualSingleShotControl();
                intakeControl();
                gate();
                setOuttakePower();
                telemetry.addData("Outtake Speed", outtake.getVelocity());
                telemetry.addData("Hood Position", hood.getPosition());
                telemetry.addData("Gate Position", gate.getPosition());
                telemetry.update();
            }
        }
    }


    private void initRobot() {
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setDirection(DcMotor.Direction.REVERSE);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        gate.setDirection(Servo.Direction.FORWARD);
        hood.setDirection(Servo.Direction.FORWARD);
        autoBlue2 = "autoBlue";
        autoRed2 = "autoRed";
        teleOp2 = "teleOp";

        nearVelocity = 1750 ;
        farVelocity = 2800;
        gateOpenPos = 0.3;
        gateClosePos = 0;
        intakePower = 0;
        hoodPosition = 0.2745;
        hood.setPosition(hoodPosition);
        gate.setPosition(gateClosePos);
    }


    private void mecanumDrive() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // Setting motor power
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    private void intakeControl() {
        if (gamepad2.touchpadWasPressed() && intakePower <= 0) {
            intake.setDirection(DcMotor.Direction.REVERSE);
            intake.setPower(1);
            intake2.setDirection(DcMotor.Direction.REVERSE);
            intake2.setPower(1);
        }
        else if(gamepad2.crossWasPressed()){
            intake.setDirection(DcMotor.Direction.FORWARD);
            intake.setPower(1);
            intake2.setDirection(DcMotor.Direction.FORWARD);
            intake2.setPower(1);
        }
        else if (gamepad2.triangleWasPressed()){
            intake.setPower(0);
            intake2.setPower(0);
        }
        intakePower = intake.getPower();
    }


    private void manualHoodControl() {
        if (gamepad2.circleWasPressed()) {
            hood.setPosition(0.2745);
        } else if (gamepad2.dpadUpWasPressed()) {
            hood.setPosition(hoodPosition + 0.02);
        } else if (gamepad2.dpadDownWasPressed()) {
            hood.setPosition(hoodPosition - 0.02);
        }
        hoodPosition = hood.getPosition();
    }


    private void nearShot() {
        targetVelocity = nearVelocity;
        if (outtake.getVelocity() >= nearVelocity - 50) {
            gate.setPosition(gateOpenPos);
        }
    }


    private void farShot() {
        targetVelocity = farVelocity;
        if (outtake.getVelocity() >= farVelocity - 50) {
            gate.setPosition(gateOpenPos);
        }
    }


    private void manualSingleShotControl() {
        if (gamepad2.rightBumperWasPressed()) {
            nearShot();
        } else if (gamepad2.leftBumperWasPressed()) {
            farShot();
        } else if (gamepad2.squareWasPressed()) {
            gate.setPosition(gateClosePos);
            targetVelocity = 0;
            outtake.setPower(0);
        }
    }

    private void gate() {

        gate.setPosition(gamepad2.right_stick_button ? gateOpenPos : gateClosePos);
    }

    private void setOuttakePower() {
        double error = targetVelocity - outtake.getVelocity();
        double power = error * .01;
        power += targetVelocity * .0005;
        outtake.setPower(power);
        }
    }