/* ============================================================= *
 *        Pedro Pathing Plus Visualizer — Auto-Generated         *
 *                                                               *
 *  Version: 1.8.0.                                              *
 *  Copyright (c) 2026 Matthew Allen                             *
 *                                                               *
 *  THIS FILE IS AUTO-GENERATED — DO NOT EDIT MANUALLY.          *
 *  Changes will be overwritten when regenerated.                *
 * ============================================================= */

package org.firstinspires.ftc.teamcode.OpModes.Auto;

import android.sax.EndElementListener;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;



@Autonomous(name = "Something Something Close Side Pathing", group = "Autonomous")
@Configurable // Panels
public class YummyDemo extends OpMode {

    private Servo gate, indicatorLight1, indicatorLight2;
    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    private DcMotorEx flywheelLeft, flywheelRight, intakeOuter, intakeInner;

    private DistanceSensor intakeSensor1, intakeSensor2;
    private boolean getLine;
    private Servo hood, raxon, laxon;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private State pathState = State.SCORE; // Current autonomous path state (state machine)
    private ElapsedTime pathTimer; // Timer for path state machine
    private Timer actionTimer;
    private Paths paths; // Paths defined in the Paths class
    private enum State{
        SCORE,
        PICKUP1,
        HUMANZONE,
        END;
    }
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        // ...
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flyL");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flyR");
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelLeft.setDirection(DcMotor.Direction.FORWARD);
        flywheelRight.setDirection(DcMotor.Direction.REVERSE);

        intakeOuter = hardwareMap.get(DcMotorEx.class, "intOuter");
        intakeInner = hardwareMap.get(DcMotorEx.class, "intInner");
        intakeOuter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeInner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeOuter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeInner.setDirection(DcMotor.Direction.REVERSE);
        intakeOuter.setDirection(DcMotor.Direction.REVERSE);
        intakeInner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //intakeSensor1 = hardwareMap.get(DistanceSensor.class, "intakeSensor1");
       // intakeSensor2 = hardwareMap.get(DistanceSensor.class, "intakeSensor2");

        hood = hardwareMap.get(Servo.class, "hood");
        raxon = hardwareMap.get(Servo.class, "raxon");
        laxon = hardwareMap.get(Servo.class, "laxon");

        follower = Constants.createFollower(hardwareMap);
        // Determine starting heading: prefer geometric heading when a path exists, otherwise fall back to explicit startPoint values
        follower.setStartingPose(
                new Pose(48.000, 10.000, Math.toRadians(-180))
        );

        pathTimer = new ElapsedTime();
        actionTimer = new Timer();
        paths = new Paths(follower); // Build paths
        pathState = State.SCORE;
        getLine = true;

    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.addData("actionTimer", actionTimer.getElapsedTimeSeconds());
        panelsTelemetry.addData("getLine", getLine);
        panelsTelemetry.addData("isBusy?", follower.isBusy());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain line1;
        public PathChain line2;
        public PathChain line3;
        public PathChain line4;
        public PathChain line5;

        public Paths(Follower follower) {
            line1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(48.000, 10.000),
                                    new Pose(24.000, 8.531),
                                    new Pose(24.000, 36.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            line2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 36.000), new Pose(48.000, 10.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            line3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 10.000), new Pose(10.000, 10.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(10.000, 10.000), new Pose(48.000, 10.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(10,10), new Pose(24,36))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(225))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case SCORE:
                raxon.setPosition(.5);
                laxon.setPosition(.5);
                flywheelLeft.setVelocity(2000);
                flywheelRight.setVelocity(2000);
                hood.setPosition(.5);

                if (follower.isBusy()){
                    actionTimer.resetTimer();
                }else{
                    intakeOuter.setPower(-.8);
                    intakeInner.setPower(.3);
                   // gate.setPosition(.88);
                    if (actionTimer.getElapsedTimeSeconds() > 3) {
                        //gate.setPosition(.5);
                        if(getLine) {
                            follower.followPath(paths.line1, true);
                            setPathState(State.PICKUP1);
                            getLine = false;
                        }
                        else
                        {
                            follower.followPath(paths.line3, true);
                            setPathState(State.HUMANZONE);
                        }

                    }
                }
            case PICKUP1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line2, true);
                    setPathState(State.SCORE);
                }
                break;
            case HUMANZONE:
                if (!follower.isBusy()) {
                    if(pathTimer.seconds() < 2)
                    {
                        follower.followPath(paths.line5);
                        setPathState(State.END);
                    }
                    else {
                        follower.followPath(paths.line4, true);
                        setPathState(State.SCORE);

                    }

                }
                break;
            case END:
                requestOpModeStop();
                break;
//            case 4:
//                follower.followPath(paths.line3, true);
//                setPathState(5);
//                break;
//            case 5:
//                if (!follower.isBusy()) {
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                follower.followPath(paths.line4, true);
//                setPathState(7);
//                break;
//            case 7:
//                if (!follower.isBusy()) {
//                    setPathState(8);
//                }
//                break;
//            case 8:
//                requestOpModeStop();
//                pathState = -1;
//                break;
        }

    }

    public void setPathState(State pState) {
        pathState = pState;
    }
}
