package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto Red Goal", group = "Autonomous")
@Configurable // Panels
public class autoRedGoal extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private DcMotor intake;
    private DcMotor intake2;
    private DcMotorEx outtake;
    private Servo gate;
    private Servo hood;
    double intakePower;
    double gateClosePos;
    double gateOpenPos;
    int nearVelocity;
    int farVelocity;
    private double targetVelocity;
    private Timer pathTimer;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        gate = hardwareMap.get(Servo.class, "gate");
        hood = hardwareMap.get(Servo.class, "hood");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setDirection(DcMotor.Direction.REVERSE);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setDirection(DcMotor.Direction.REVERSE);
        gate.setDirection(Servo.Direction.FORWARD);
        gate.setPosition(0.3);
        hood.setDirection(Servo.Direction.FORWARD);
        hood.setPosition(0.2745);
        nearVelocity = 1750 ;
        farVelocity = 2800;
        gateOpenPos = 0.1;
        gateClosePos = 0.3;
        intakePower = 0;
        targetVelocity = 0;

        pathTimer = new Timer();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(123.538, 122.774, Math.toRadians(216)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }
    // a place to put your intake and shooting functions
    public void intakeArtifacts() {
        // Put your intake logic/functions here
        intake.setPower(1);
        intake2.setPower(1);
    }
    public void shootArtifacts() {
        // Put your shooting logic/functions here
        gate.setPosition(gateOpenPos);
        //if(pathTimer.getElapsedTimeSeconds() > 6){
          //  gate.setPosition(gateClosePos);
        //}
    }


    private void setOuttakePower() {
        double error = targetVelocity - outtake.getVelocity();
        double power = error * .01;
        power += targetVelocity * .0005;
        outtake.setPower(power);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine
        setOuttakePower();

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());


        telemetry.addData("Gate Position",gate.getPosition());
        telemetry.addData("Path Timer",pathTimer.getElapsedTime());
        telemetry.update();
        panelsTelemetry.update(telemetry);

    }

    public static class Paths {

        public PathChain scorePreload;
        public PathChain rotatePPG;
        public PathChain grabPPG;
        public PathChain scorePPG;
        public PathChain rotatePGP;
        public PathChain grabPGP;
        public PathChain scorePGP;
        public PathChain rotateGPP;
        public PathChain grabGPP;
        public PathChain scoreGPP;
        public PathChain park;

        public Paths(Follower follower) {
            scorePreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(123.538, 122.774), new Pose(82.839, 84.968))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(216), Math.toRadians(225))
                    .build();

            rotatePPG = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(82.839, 84.968), new Pose(85.161, 86.903))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                    .build();

            grabPPG = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(85.161, 86.903), new Pose(122.129, 87.290))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            scorePPG = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(122.129, 87.290), new Pose(82.839, 84.968))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                    .build();

            rotatePGP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(82.839, 84.968), new Pose(90.387, 63.871))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                    .build();

            grabPGP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(90.387, 63.871), new Pose(122.129, 63.871))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            scorePGP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(122.129, 63.871), new Pose(82.839, 85.161))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                    .build();

            rotateGPP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(82.839, 85.161), new Pose(98.903, 39.097))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                    .build();

            grabGPP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(98.903, 39.097), new Pose(130.839, 39.097))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            scoreGPP = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(130.839, 39.097), new Pose(82.839, 85.161))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                    .build();

            park = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(82.839, 85.161), new Pose(125.419, 98.516))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(180))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine here
        // Access paths with paths.pathName
        switch (pathState) {
            case 0:
                intakeArtifacts();
                targetVelocity = nearVelocity;
                if(pathTimer.getElapsedTimeSeconds() >= 4.5) {
                    follower.followPath(paths.scorePreload);
                    setPathState(1);
                }
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preloads */
                    shootArtifacts();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the artifact */
                    if(pathTimer.getElapsedTimeSeconds() >= 4){
                        gate.setPosition(gateClosePos);
                        follower.followPath(paths.rotatePPG);
                        setPathState(2);
                    }

                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Artifact */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the artifact */
                    follower.setMaxPower(0.5);
                    follower.followPath(paths.grabPPG);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(paths.scorePPG);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    /* Score PPG */
                    shootArtifacts();
                    if(pathTimer.getElapsedTimeSeconds() >= 4) {
                        gate.setPosition(gateClosePos);
                        follower.followPath(paths.rotatePGP);
                        setPathState(5);
                    }

                }
                break;
            case 5:
                if(!follower.isBusy()){
                    follower.setMaxPower(0.5);
                    follower.followPath(paths.grabPGP);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(paths.scorePGP);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    /* Score PGP */
                    targetVelocity = nearVelocity;
                    shootArtifacts();
                    if(pathTimer.getElapsedTimeSeconds() >= 4) {
                        gate.setPosition(gateClosePos);
                        follower.followPath(paths.rotateGPP);
                        setPathState(8);
                    }

                }
                break;
            case 8:
                if(!follower.isBusy()){
                    follower.followPath(paths.grabGPP);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()){
                    follower.followPath(paths.scoreGPP);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()){
                    /* Score GPP */
                    targetVelocity = nearVelocity;
                    shootArtifacts();
                    if(pathTimer.getElapsedTimeSeconds() >= 4) {
                        targetVelocity = 0;
                        intake.setPower(0);
                        intake2.setPower(0);
                        gate.setPosition(gateClosePos);
                        follower.followPath(paths.park);
                        setPathState(-1);
                    }

                }
                break;
        }
        return pathState;
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
    }
}
