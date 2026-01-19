package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto Blue Far", group = "Autonomous")
@Configurable // Panels
public class autoBlueFar extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private DcMotor intake;
    private DcMotor intake2;
    private DcMotorEx outtake;
    private Servo gate;
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
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setDirection(DcMotor.Direction.REVERSE);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setDirection(DcMotor.Direction.REVERSE);
        gate.setDirection(Servo.Direction.FORWARD);
        gate.setPosition(0.3);
        nearVelocity = 1750 ;
        farVelocity = 2800;
        gateOpenPos = 0.3;
        gateClosePos = 0;
        intakePower = 0;
        targetVelocity = 0;

        pathTimer = new Timer();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(57.355, 8.580, Math.toRadians(270)));

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
        public PathChain Path2;

        public Paths(Follower follower) {
            scorePreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.355, 8.581),

                                    new Pose(65.871, 16.065)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(290))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(65.871, 16.065),

                                    new Pose(36.548, 14.581)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(180))

                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine here
        // Access paths with paths.pathName
        switch (pathState) {
            case 0:

                targetVelocity = farVelocity;
                if(pathTimer.getElapsedTimeSeconds() >= 7) {
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
                    if(pathTimer.getElapsedTimeSeconds() >= 10){
                        gate.setPosition(gateClosePos);
                        targetVelocity = 0;
                        intake.setPower(0);
                        intake2.setPower(0);
                        follower.followPath(paths.Path2);
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
