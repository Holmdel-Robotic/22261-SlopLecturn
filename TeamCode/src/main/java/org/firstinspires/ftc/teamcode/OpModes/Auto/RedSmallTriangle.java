package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.dashboard.config.Config;
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

//
@Autonomous(name = "Red  Small Triangle", group = "Autonomous")
@Config
public class RedSmallTriangle extends OpMode {

    private Follower follower;
    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor, intakeOuter, intakeInner, flywheelLeft, flywheelRight;

    private Servo hood, raxon, laxon, innerGate, outerGate;

    public static double VELO = 2200, SERVOPOS = .72
            , HOODPOS = .375, ADJUSTEDSERVOPOS = .465, loops = 0;

    private TelemetryManager panelsTelemetry;
    private Timer timer2;
    private ElapsedTime pathTimer;
    private Timer actionTimer;
    private Paths paths;

    private boolean scored = false, firstTime = true;
    private boolean pickedUp = false;

    private enum State {
        START,

        FLYWHEELRAMPUP,
        SCORE,
        PICKUP1,
        HUMANZONE,
        END;
    }

    private State pathState = State.START;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

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
        intakeInner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeInner.setDirection(DcMotor.Direction.REVERSE);
        intakeOuter.setDirection(DcMotor.Direction.FORWARD);

        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flyL");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flyR");
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelLeft.setDirection(DcMotor.Direction.FORWARD);
        flywheelRight.setDirection(DcMotor.Direction.REVERSE);

        hood = hardwareMap.get(Servo.class, "hood");
        raxon = hardwareMap.get(Servo.class, "raxon");
        laxon = hardwareMap.get(Servo.class, "laxon");
        raxon.setPosition(ADJUSTEDSERVOPOS);
        laxon.setPosition(ADJUSTEDSERVOPOS);
        innerGate = hardwareMap.get(Servo.class, "innerGate");
        outerGate = hardwareMap.get(Servo.class, "outerGate");
        outerGate.setPosition(0);
        hood.setDirection(Servo.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(mirror(new  Pose(47.800, 9, Math.toRadians(90))));
        timer2 = new Timer();
        pathTimer = new ElapsedTime();
        actionTimer = new Timer();
        paths = new Paths(follower);
        pathState = State.START;
        innerGate.setPosition(.2);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("flywheelV", flywheelLeft.getVelocity());
        telemetry.addData("FirstTime", firstTime);
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("actionTimer", actionTimer.getElapsedTimeSeconds());
        panelsTelemetry.debug("isBusy?", follower.isBusy());
        panelsTelemetry.debug("pathTime", pathTimer.seconds());
        panelsTelemetry.update(telemetry);
    }





        public static class Paths {
            public PathChain line1;
            public PathChain line2;
            public PathChain line3;
            public PathChain line4;
            public PathChain line5;

            public Paths(Follower follower) {
                line1 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(97.000, 9.000),
                                        new Pose(119.571, 12.216),
                                        new Pose(119.469, 35.318)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                        .build();

                line2 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(119.469, 35.318),

                                        new Pose(97.488, 8.645)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                        .build();

                line3 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(97.488, 8.645),

                                        new Pose(133.943, 9.569)
                                )
                        ).setTangentHeadingInterpolation()

                        .build();

                line4 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(133.943, 9.569),

                                        new Pose(97.109, 9.569)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                line5 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(97.109, 9.569),

                                        new Pose(84.033, 36.047)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();
            }
        }




        public static Pose mirror(Pose p) {
        return new Pose(144 - p.getX(), p.getY(), Math.toRadians(180 - Math.toDegrees(p.getHeading())));
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case START:
                innerGate.setPosition(.2);
                raxon.setPosition(ADJUSTEDSERVOPOS);
                laxon.setPosition(ADJUSTEDSERVOPOS);
                flywheelLeft.setVelocity(VELO);
                flywheelRight.setVelocity(VELO);
                intakeOuter.setPower(.9);
                setPathState(State.FLYWHEELRAMPUP);
                actionTimer.resetTimer();
                timer2.resetTimer();
                outerGate.setPosition(0);
                break;

            case FLYWHEELRAMPUP:
                if (actionTimer.getElapsedTimeSeconds() > 4){
                    actionTimer.resetTimer();
                    setPathState(State.SCORE);
                    innerGate.setPosition(.575);
                    outerGate.setPosition(0);
                }
                break;

            case SCORE:
                if (timer2.getElapsedTimeSeconds() > 1){
                    outerGate.setPosition(.6);
                    intakeInner.setPower(.9);
                }

                if (follower.isBusy() || firstTime) {
                    actionTimer.resetTimer();
                    firstTime = false;
                }

                else {

                    if (!pickedUp){
                        raxon.setPosition(ADJUSTEDSERVOPOS);
                        laxon.setPosition(ADJUSTEDSERVOPOS);
                    }else{
                        raxon.setPosition(SERVOPOS);
                        laxon.setPosition(SERVOPOS);
                    }


                    flywheelLeft.setVelocity(VELO);
                    flywheelRight.setVelocity(VELO);
                    hood.setPosition(HOODPOS);
                    innerGate.setPosition(.575);
                    outerGate.setPosition(.6);
                    intakeInner.setPower(.9);
                    if (actionTimer.getElapsedTimeSeconds() > 2) {

                        intakeInner.setPower(0);
                        innerGate.setPosition(.2);
                        outerGate.setPosition(0);
                        if (!pickedUp) {
                            follower.followPath(paths.line1, true);
                            setPathState(State.PICKUP1);

                        }
                        else if (loops >= 5){
                            follower.followPath(paths.line5);
                            setPathState(State.END);
                        }
                        else {
                            loops += 1;
                            follower.followPath(paths.line3, true);
                            setPathState(State.HUMANZONE);
                        }
                    }
                }
                break;

            case PICKUP1:
                if (!follower.isBusy()) {
                    pickedUp = true;
                    follower.followPath(paths.line2, true);
                    setPathState(State.SCORE);
                }
                break;

            case HUMANZONE:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line4, true);
                    setPathState(State.SCORE);
                    timer2.resetTimer();
                }
                break;

            case END:
                if (!follower.isBusy()) {
                    requestOpModeStop();
                }
                break;
        }
    }

    public void setPathState(State pState) {
        pathState = pState;
        pathTimer.reset();
    }
}
