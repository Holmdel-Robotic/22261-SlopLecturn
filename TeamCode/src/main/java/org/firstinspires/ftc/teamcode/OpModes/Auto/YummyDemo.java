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

@Autonomous(name = "YummyDemo",   group = "Autonomous")
@Configurable // Panels
public class YummyDemo extends OpMode {

    private Servo hood;

    private Servo raxon;

    private Servo laxon;

    private DcMotorEx flywheelLeft;

    private DcMotorEx flywheelRight;

    private DcMotorEx IntakeInner;

    private DcMotorEx IntakeOuter;
    private Servo kicker;
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
    public int count = 0;

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
        flywheelLeft.setDirection(DcMotor.Direction.REVERSE);
        flywheelRight.setDirection(DcMotor.Direction.FORWARD);

        raxon = hardwareMap.get(Servo.class,"raxon");
        laxon = hardwareMap.get(Servo.class,"laxon");

        IntakeInner = hardwareMap.get(DcMotorEx.class, "intInner");
        IntakeOuter = hardwareMap.get(DcMotorEx.class, "intOuter");
        kicker = hardwareMap.get(Servo.class, "blocker");
        IntakeInner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeOuter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeInner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeOuter.setDirection(DcMotor.Direction.REVERSE);
        IntakeInner.setDirection(DcMotor.Direction.FORWARD);
        IntakeOuter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pathState = State.START;

        hood = hardwareMap.get(Servo.class, "hood");


        raxon.setPosition(.63);
        laxon.setPosition(.63);
        kicker.setPosition(.5);

        follower = Constants.createFollower(hardwareMap);
        // Determine starting heading: prefer geometric heading when a path exists, otherwise fall back to explicit startPoint values
        follower.setStartingPose(new Pose(24.000, 120.000, Math.toRadians(90.000)));

        pathTimer = new ElapsedTime();
        actionTimer = new Timer();
        paths = new Paths(follower); // Build paths
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

    /* ============================================================= *
     *        Pedro Pathing Plus Visualizer — Auto-Generated         *
     *                                                               *
     *  Version: 1.8.0.                                              *
     *  Copyright (c) 2026 Matthew Allen                             *
     *                                                               *
     *  THIS FILE IS AUTO-GENERATED — DO NOT EDIT MANUALLY.          *
     *  Changes will be overwritten when regenerated.                *
     * ============================================================= */

    public static class Paths {

        public PathChain line1;
        public PathChain line2;
        public PathChain line3;
        public PathChain line4;
        public PathChain line5;
        public PathChain line6;
        public PathChain line7;

        public Paths(Follower follower) {
            line1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 120.000), new Pose(50.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            line2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(50.000, 84.000), new Pose(24.000, 84.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 84.000), new Pose(60.000, 75.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            line4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(60.000, 75.000),
                                    new Pose(60.000, 60.000),
                                    new Pose(24.000, 60.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            line5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 60.000), new Pose(56.000, 82.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            line6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 82.000), new Pose(10.000, 60.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(135))
                    .build();

            line7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(10.000, 60.000), new Pose(56.000, 82.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(135))
                    .build();
        }
    }



    public void autonomousPathUpdate(State state) {
        switch (state) {
            case START:
                //if(!follower.isBusy())
                //{
                flywheelLeft.setVelocity(-1675);
                flywheelRight.setVelocity(-1675);
                hood.setPosition(.67);
                kicker.setPosition(.3);
                IntakeOuter.setPower(-.8);
                IntakeInner.setPower(-.4);

                actionTimer.resetTimer();
                follower.followPath(paths.line1, true);
                setPathState(State.SCORE);
                break;
            //}

            case SCORE:
                if(follower.isBusy())
                {
                    actionTimer.resetTimer();
                }
                else if (!follower.isBusy())
                {

                    if(actionTimer.getElapsedTimeSeconds() <= 1){


                        hood.setPosition(.67);
                        // && flywheelRight.getVelocity() > 1650 && flywheelLeft.getVelocity() > 1650 &&
                    } else if (actionTimer.getElapsedTimeSeconds() >= 1 && actionTimer.getElapsedTimeSeconds() <= 3) {

                        IntakeOuter.setPower(-.8);
                        IntakeInner.setPower(-.4);
                        kicker.setPosition(.5);


                    }
                    else if (actionTimer.getElapsedTimeSeconds() >= 3 && actionTimer.getElapsedTimeSeconds() <= 3.1) {
                        kicker.setPosition(.3);


                    }
                    else {

                        //IntakeInner.setVelocity(0);
                        //IntakeOuter.setVelocity(0);
//                        flywheelLeft.setVelocity(-.01);
//                        flywheelRight.setVelocity(-.01);
                        if (count == 1) {

                            follower.followPath(paths.line2, true);
                            setPathState(State.PICKUP1);
                            count++;
                        } else if (count == 2) {
                            follower.followPath(paths.line4, true);
                            setPathState(State.PICKUP2);
                            count++;
                        } else if (count == 3) {
                            follower.followPath(paths.line6, true);
                            setPathState(State.PICKUP3);
                            count++;

                        }
//                        else if(count == 4)
//                        {
//                            follower.followPath(paths.Path8, true);
//                            setPathState(State.PICKUP4);
//                            count++;
//                        }
//                        else
//                        {
//                            follower.followPath(paths.Path10, true);
//                            setPathState(State.END);
//                        }

                    }
                }


                break;
            case PICKUP1:
                if(!follower.isBusy()) {
                    follower.followPath(paths.line3, true);

                    setPathState(State.SCORE);
                }
                break;
            case PICKUP2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line5);
                    setPathState(State.SCORE);
                }
                break;
            case PICKUP3:
                if(!follower.isBusy()){
                    follower.followPath(paths.line7, true);
                    setPathState(State.END);
                }

                break;
//            case PICKUP4:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path9, true);
//                    setPathState(State.SCORE);
//                }
//                break;
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
