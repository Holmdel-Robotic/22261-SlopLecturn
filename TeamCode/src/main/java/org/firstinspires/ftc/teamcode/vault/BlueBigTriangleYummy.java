
package org.firstinspires.ftc.teamcode.vault;
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

@Autonomous(name = "Blue Big Triangle Yummy", group = "Autonomous")
@Configurable // Panels
public class BlueBigTriangleYummy extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private State pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor, intakeOuter, intakeInner, flywheelLeft, flywheelRight;

    private Servo hood, raxon, laxon, innerGate, outerGate;

    private Timer actionTimer;
    private boolean scored = false;
    private int row = 0;

    private enum State {
        START,
        SCORE,
        COLLECT,
        END;
    }



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
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));
        actionTimer = new Timer();
        paths = new Paths(follower); // Build paths
        pathState = State.START;
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
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
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain Path1;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;

        public PathChain Path2;




        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.500, 120.400),

                                    new Pose(48.600, 84.700)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.600, 84.700),

                                    new Pose(19.000, 84.700)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.000, 84.700),

                                    new Pose(48.600, 84.700)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.600, 84.700),
                                    new Pose(43.092, 57.427),
                                    new Pose(23.855, 60.169)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.855, 60.169),

                                    new Pose(48.600, 84.700)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.600, 84.700),

                                    new Pose(12.300, 60.700)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.300, 60.700),

                                    new Pose(48.600, 84.700)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(180))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.600, 84.700),

                                    new Pose(12.300, 60.700)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.300, 60.700),

                                    new Pose(48.600, 84.700)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(180))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.600, 84.700),

                                    new Pose(60.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();
        }
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case START:
                actionTimer.resetTimer();
                raxon.setPosition(.5);
                laxon.setPosition(.5);
                flywheelLeft.setVelocity(2000);
                flywheelRight.setVelocity(2000);
                intakeOuter.setPower(.9);
                intakeInner.setPower(.8);
                follower.followPath(paths.Path1, true);
                setPathState(State.SCORE);
                outerGate.setPosition(.6);
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
                        if (row == 0) {
                            follower.followPath(paths.Path2, true);
                            row++;
                            setPathState(State.COLLECT);
                        }
                        else if (row == 1) {
                            follower.followPath(paths.Path4, true);
                            row++;
                            setPathState(State.COLLECT);
                        }
                        else if (row == 2) {
                            follower.followPath(paths.Path6, true);
                            row++;
                            setPathState(State.COLLECT);
                        }
                        else if (row == 3){
                            follower.followPath(paths.Path8, true);
                            row++;
                            setPathState(State.COLLECT);
                        }
                        else
                        {
                            follower.followPath(paths.Path10, true);
                            setPathState(State.END);
                        }
                    }
                }
                break;

            case COLLECT:
                if (!follower.isBusy()) {
                    if (row == 1) {
                        follower.followPath(paths.Path3, true);
                    }
                    else if (row == 2) {
                        follower.followPath(paths.Path5, true);
                    }
                    else if(row == 3) {
                        follower.followPath(paths.Path7, true);
                    }
                    else {
                        follower.followPath(paths.Path9, true);
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
//        pathTimer.reset();
    }
}
    