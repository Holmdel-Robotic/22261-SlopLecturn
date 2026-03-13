/* ============================================================= *
 *        Pedro Pathing Plus Visualizer — Auto-Generated         *
 *                                                               *
 *  Version: 1.7.4.                                              *
 *  Copyright (c) 2026 Matthew Allen                             *
 *                                                               *
 *  THIS FILE IS AUTO-GENERATED — DO NOT EDIT MANUALLY.          *
 *  Changes will be overwritten when regenerated.                *
 * ============================================================= */

package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "OptimizedBluePathing")
@Configurable // Panels
public class OptimizedBlue_IN_PROGRESS extends OpMode {

    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor, intakeOuter,intakeInner, flywheelLeft, flywheelRight;

    private Servo hood, raxon, laxon, innerGate, outerGate;
    
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance

     // Current autonomous path state (state machine)
    private ElapsedTime pathTimer;

    private Timer actionTimer;// Timer for path state machine
    private Paths paths; // Paths defined in the Paths class

    private enum State{
        START,
        SCORE,
        PICKUP1,
        PICKUP2,
        PICKUP3,
        PICKUP4,
        END
    }
    State pathState;
    public int count = 1;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        // ...
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(144-84, 36, Math.toRadians(180)));
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fl");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fr");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "bl");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "br");
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeOuter = hardwareMap.get(DcMotorEx.class, "intOuter");
        intakeInner = hardwareMap.get(DcMotorEx.class, "intInner");
        intakeOuter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeInner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeOuter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeInner.setDirection(DcMotor.Direction.REVERSE);
        intakeOuter.setDirection(DcMotor.Direction.FORWARD);
        intakeInner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flyL");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flyR");
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelLeft.setDirection(DcMotor.Direction.FORWARD);
        flywheelRight.setDirection(DcMotor.Direction.REVERSE);

        hood = hardwareMap.get(Servo.class, "hood");
        raxon = hardwareMap.get(Servo.class, "raxon");
        laxon = hardwareMap.get(Servo.class, "laxon");
        innerGate = hardwareMap.get(Servo.class, "innerGate");
        outerGate = hardwareMap.get(Servo.class, "outerGate");
        hood.setDirection(Servo.Direction.REVERSE);

        //intakeSensor1 = hardwareMap.get(DistanceSensor.class, "intakeSensor1");
        // intakeSensor2 = hardwareMap.get(DistanceSensor.class, "intakeSensor2");

        hood = hardwareMap.get(Servo.class, "hood");
        raxon = hardwareMap.get(Servo.class, "raxon");
        laxon = hardwareMap.get(Servo.class, "laxon");

        follower = Constants.createFollower(hardwareMap);
        // Determine starting heading: prefer geometric heading when a path exists, otherwise fall back to explicit startPoint values
        follower.setStartingPose(
                new Pose(24, 120.000, Math.toRadians(90))
        );

        pathTimer = new ElapsedTime();
        actionTimer = new Timer();
        paths = new Paths(follower); // Build paths
        pathState = State.START; // Build paths
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(pathState); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;


        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 120.000), new Pose(48.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 84.000), new Pose(24.000, 84.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 84.000), new Pose(56.000, 74.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.000, 74.000),
                                    new Pose(48.000, 62.000),
                                    new Pose(14.000, 64.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(14.000, 64.000), new Pose(52.000, 12.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(52.000, 12.000),
                                    new Pose(24.000, 9.000),
                                    new Pose(24.000, 36.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 36.000), new Pose(52.000, 12.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(52.000, 12.000), new Pose(8.000, 10.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(8.000, 10.000), new Pose(56.000, 18.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 18.000), new Pose(8.000, 18.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }


    public void autonomousPathUpdate(State state) {
        switch (state) {
            case START:
                //if(!follower.isBusy())
                //{
                raxon.setPosition(.5);
                laxon.setPosition(.5);
                flywheelLeft.setVelocity(2000);
                flywheelRight.setVelocity(2000);
                intakeOuter.setPower(.9);

                    actionTimer.resetTimer();
                    follower.followPath(paths.Path1, true);
                    setPathState(State.SCORE);
                    break;
                //}

            case SCORE:
                if (follower.isBusy()){
                    actionTimer.resetTimer();
                }
                else
                {
                    intakeInner.setPower(.9);
                    innerGate.setPosition(.575);
                    outerGate.setPosition(.6);
//                    gate.setPosition(.88);
                    if (actionTimer.getElapsedTimeSeconds() > 2) {
//                        gate.setPosition(.5);
                        intakeInner.setPower(0);
                        innerGate.setPosition(.2);
                        outerGate.setPosition(0);
//                if(follower.isBusy())
//                {
//                    actionTimer.resetTimer();
//                }
//                else if (!follower.isBusy())
//                {
//
//                    if(actionTimer.getElapsedTimeSeconds() <= 1){
//
//
//                        hood.setPosition(.67);
//                        // && flywheelRight.getVelocity() > 1650 && flywheelLeft.getVelocity() > 1650 &&
//                    } else if (actionTimer.getElapsedTimeSeconds() >= 1 && actionTimer.getElapsedTimeSeconds() <= 3) {
//
//                        IntakeOuter.setPower(-.8);
//                        IntakeInner.setPower(-.4);
//                        kicker.setPosition(.5);
//
//
//                    }
//                    else if (actionTimer.getElapsedTimeSeconds() >= 3 && actionTimer.getElapsedTimeSeconds() <= 3.1) {
//                        kicker.setPosition(.3);
//
//
//                    }
//                    else {

                        //IntakeInner.setVelocity(0);
                        //IntakeOuter.setVelocity(0);
//                        flywheelLeft.setVelocity(-.01);
//                        flywheelRight.setVelocity(-.01);
                        if (count == 1) {

                            follower.followPath(paths.Path2, true);
                            setPathState(State.PICKUP1);
                            count++;
                        } else if (count == 2) {
                            follower.followPath(paths.Path4, true);
                            setPathState(State.PICKUP2);
                            count++;
                        } else if (count == 3) {
                            follower.followPath(paths.Path6, true);
                            setPathState(State.PICKUP3);
                            count++;

                        }
                        else if(count == 4)
                        {
                            follower.followPath(paths.Path8, true);
                            setPathState(State.PICKUP4);
                            count++;
                        }
                        else
                        {
                            follower.followPath(paths.Path10, true);
                            setPathState(State.END);
                        }

                    }
                }


                break;
            case PICKUP1:
                if(!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);

                    setPathState(State.SCORE);
                }
                break;
            case PICKUP2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5);
                    setPathState(State.SCORE);
                }
                break;
            case PICKUP3:
                if(!follower.isBusy()){
                    follower.followPath(paths.Path7, true);
                    setPathState(State.SCORE);
                }

                break;
            case PICKUP4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9, true);
                    setPathState(State.SCORE);
                }
                break;
            case END:
                if(!follower.isBusy()){
                    requestOpModeStop();
                }
//            case 6:
//                follower.followPath(paths.Path4, true);
//                setPathState(7);
//                break;
//            case 7:
//                if (!follower.isBusy()) {
//                    setPathState(8);
//                }
//                break;
//            case 8:
//                follower.followPath(paths.Path5, true);
//                setPathState(9);
//                break;
//            case 9:
//                if (!follower.isBusy()) {
//                    setPathState(10);
//                }
//                break;
//            case 10:
//                follower.followPath(paths.Path6, true);
//                setPathState(11);
//                break;
//            case 11:
//                if (!follower.isBusy()) {
//                    setPathState(12);
//                }
//                break;
//            case 12:
//                follower.followPath(paths.Path7, true);
//                setPathState(13);
//                break;
//            case 13:
//                if (!follower.isBusy()) {
//                    setPathState(14);
//                }
//                break;
//            case 14:
//                follower.followPath(paths.Path8, true);
//                setPathState(15);
//                break;
//            case 15:
//                if (!follower.isBusy()) {
//                    setPathState(16);
//                }
//                break;
//            case 16:
//                follower.followPath(paths.Path9, true);
//                setPathState(17);
//                break;
//            case 17:
//                if (!follower.isBusy()) {
//                    setPathState(18);
//                }
//                break;
//            case 18:
//                follower.followPath(paths.Path10, true);
//                setPathState(19);
//                break;
//            case 19:
//                if (!follower.isBusy()) {
//                    setPathState(20);
//                }
//                break;
//            case 20:
//                requestOpModeStop();
//                pathState = -1;
//                break;
        }

    }
    public void setPathState(State pState) {
        pathState = pState;
        pathTimer.reset();
    }
}
