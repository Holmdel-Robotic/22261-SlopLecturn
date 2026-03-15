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

@Autonomous(name = "Blue Big Triangle", group = "Autonomous")
@Configurable
public class BlueBigTriangle extends OpMode {

    private Follower follower;
    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor, intakeOuter, intakeInner, flywheelLeft, flywheelRight;

    private Servo hood, raxon, laxon, innerGate, outerGate;

    private TelemetryManager panelsTelemetry;
    private ElapsedTime pathTimer;
    private Timer actionTimer;
    private Paths paths;

    private double SERVOPOS = .3, FLYV = 1600, HOOD =.2;

    private boolean scored = false;
    private int row = 0;

    private enum State {
        START,
        SCORE,
        COLLECT,
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
        follower.setStartingPose(new Pose(22.500, 120.400, Math.toRadians(180)));

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
        panelsTelemetry.debug("row", row);
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain line1;
        public PathChain line2;
        public PathChain line3;
        public PathChain line4;
        public PathChain line5;
        public PathChain line6;
        public PathChain line7;
        public PathChain line8;

        public Paths(Follower follower) {
            line1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(22.500, 120.400), new Pose(48.688, 84.745))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            line2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.688, 84.745), new Pose(21.271, 84.303))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            line3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(21.271, 84.303), new Pose(48.489, 84.592))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            line4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(48.489, 84.592),
                                    new Pose(55.953, 57.344),
                                    new Pose(19.250, 60.080)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            line5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(19.250, 60.080), new Pose(48.633, 84.592))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            line6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(48.633, 84.592),
                                    new Pose(52.549, 30.829),
                                    new Pose(19.766, 35.262)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            line7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(19.766, 35.262), new Pose(48.506, 84.317))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            line8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.506, 84.317), new Pose(60.000, 36.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case START:
                raxon.setPosition(SERVOPOS);
                laxon.setPosition(SERVOPOS);
                flywheelLeft.setVelocity(FLYV);
                flywheelRight.setVelocity(FLYV);
                hood.setPosition(HOOD);
                intakeOuter.setPower(.9);
                intakeInner.setPower(.8);
                follower.followPath(paths.line1, true);
                setPathState(State.SCORE);
                resetRuntime();
                break;

            case SCORE:
                if (getRuntime() > 1 && follower.isBusy()){
                    outerGate.setPosition(.6);
                }
                if (follower.isBusy()) {
                    actionTimer.resetTimer();
                }
                else {
                    innerGate.setPosition(.575);
                    outerGate.setPosition(.6);
                    if (actionTimer.getElapsedTimeSeconds() > 1) {
                        intakeInner.setPower(0);
                        innerGate.setPosition(.2);
                        outerGate.setPosition(0);
                        if (row == 0) {
                            follower.followPath(paths.line2, true);
                            row++;
                            setPathState(State.COLLECT);
                        }
                        else if (row == 1) {
                            follower.followPath(paths.line4, true);
                            row++;
                            setPathState(State.COLLECT);
                        }
                        else if (row == 2) {
                            follower.followPath(paths.line6, true);
                            row++;
                            setPathState(State.COLLECT);
                        }
                        else {
                            follower.followPath(paths.line8, true);
                            setPathState(State.END);
                        }
                    }
                }
                break;

            case COLLECT:
                if (!follower.isBusy()) {
                    if (row == 1) {
                        follower.followPath(paths.line3, true);
                    }
                    else if (row == 2) {
                        follower.followPath(paths.line5, true);
                    }
                    else {
                        follower.followPath(paths.line7, true);
                    }
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
    