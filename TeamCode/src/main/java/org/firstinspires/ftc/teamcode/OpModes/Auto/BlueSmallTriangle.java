package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//
@Autonomous(name = "Blue  Small Triangle", group = "Autonomous")
@Configurable
public class BlueSmallTriangle extends OpMode {

    private Follower follower;
    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor, intakeOuter, intakeInner, flywheelLeft, flywheelRight;

    private Servo hood, raxon, laxon, innerGate, outerGate;

    private TelemetryManager panelsTelemetry;
    private ElapsedTime pathTimer;
    private Timer actionTimer;
    private Paths paths;

    private boolean scored = false;
    private boolean pickedUp = false;

    private enum State {
        START,
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
        innerGate = hardwareMap.get(Servo.class, "innerGate");
        outerGate = hardwareMap.get(Servo.class, "outerGate");
        hood.setDirection(Servo.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(47.800, 12.500, Math.toRadians(90)));

        pathTimer = new ElapsedTime();
        actionTimer = new Timer();
        paths = new Paths(follower);
        pathState = State.START;
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

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
            line1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(47.800, 13.000),
                                    new Pose(22.203, 23.234),
                                    new Pose(24.000, 33.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(100))
                    .build();

            line2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(24.000, 33.000),
                                    new Pose(22.609, 23.031),
                                    new Pose(47.800, 13.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(100), Math.toRadians(180))
                    .build();

            line3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.800, 13.000),
                                    new Pose(6.700, 13.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            line4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(6.700, 13.000),
                                    new Pose(47.800, 13.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            line5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.800, 13.000),
                                    new Pose(6.000, 36.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case START:
                raxon.setPosition(.5);
                laxon.setPosition(.5);
                flywheelLeft.setVelocity(2000);
                flywheelRight.setVelocity(2000);
                intakeOuter.setPower(.9);
                setPathState(State.SCORE);
                break;

            case SCORE:
                if (follower.isBusy()) {
                    actionTimer.resetTimer();
                }
                else {
                    innerGate.setPosition(.575);
                    outerGate.setPosition(.6);
                    if (actionTimer.getElapsedTimeSeconds() > 2) {
                        intakeInner.setPower(0);
                        innerGate.setPosition(.2);
                        outerGate.setPosition(0);
                        if (!pickedUp) {
                            follower.followPath(paths.line1, true);
                            setPathState(State.PICKUP1);
                        }
                        else if (pathTimer.seconds() > 29.7) {
                            follower.followPath(paths.line5);
                            setPathState(State.END);
                        }
                        else {
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
