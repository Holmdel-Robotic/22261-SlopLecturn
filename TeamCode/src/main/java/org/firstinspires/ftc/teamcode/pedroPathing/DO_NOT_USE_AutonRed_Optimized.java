package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutonRed_Optimized", group = "Examples")
public class DO_NOT_USE_AutonRed_Optimized extends OpMode {

    private Servo hood;

    private Servo raxon;

    private Servo laxon;

    private DcMotorEx flywheelLeft;

    private DcMotorEx flywheelRight;

    private DcMotorEx IntakeInner;

    private DcMotorEx IntakeOuter;


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int count;
    private final Pose startPose = new Pose(115, 128, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose score1 = new Pose(94.5, 94.3, Math.toRadians(40));


    private final Pose toPickup1 = new Pose(108.92857142857143, 83.7857142857143, Math.toRadians(0)); // Highest (First Set) of Artifacts.
//    private final Pose pickup3Pose = new Pose(42, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.

    private final Pose toPickupC1 = new Pose(99.39285714285714, 83.64999999999999);
    private final Pose pickup1 = new  Pose(130.85714285714286, 83.7857142857143, Math.toRadians(0));

    private final Pose score2 = new Pose(86, 83.85714285714285, Math.toRadians(0));

    private final Pose toPickup2 = new Pose(107.85714285714285, 59);

    private final Pose toPickupC2 = new Pose(96.67857142857144, 57.28571428571429);
    private final Pose pickup2 = new Pose(129.2857142857143, 58.78571428571429, Math.toRadians(0)); // Second Row of Artifacts from the Spike Mark.
// changed 124 to 126
    private final Pose score3 = new Pose(98,78);

    private final Pose toPickup3 = new Pose(106,35);

    private final Pose toPickupC3 = new Pose(89.53571428571426,33.142857142857125);

    private final Pose pickup3 = new Pose(130.07142857142858, 35.42857142857143, Math.toRadians(0));

    private final Pose score4 = new Pose(89, 77.50000000000001);

    private final Pose pickup4 = new Pose(137.85714285714283,10.214285714285726);

    private final Pose score5 = new Pose(87.07142857142857, 116.00000000000003);

    private Path scorePreload;

    private PathChain toG1, g1, s1, toG2, g2, s2, toG3, g3, s3, g4, s4 ;

    private Servo kicker;





    enum State {
        START,
        SCORE,
        TRAVEL,
        PICKUP,
        END

    }

    State state = State.SCORE;
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, score1));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), score1.getHeading());

        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flyL");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flyR");
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelLeft.setDirection(DcMotor.Direction.REVERSE);
        flywheelRight.setDirection(DcMotor.Direction.FORWARD);

        raxon = hardwareMap.get(Servo.class, "raxon");
        laxon = hardwareMap.get(Servo.class, "laxon");

        IntakeInner = hardwareMap.get(DcMotorEx.class, "intInner");
        IntakeOuter = hardwareMap.get(DcMotorEx.class, "intOuter");
        kicker = hardwareMap.get(Servo.class, "blocker");
        IntakeInner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeOuter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeInner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeOuter.setDirection(DcMotor.Direction.REVERSE);
        IntakeInner.setDirection(DcMotor.Direction.FORWARD);
        IntakeOuter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        hood = hardwareMap.get(Servo.class, "hood");


        raxon.setPosition(.34);
        laxon.setPosition(.34);
        kicker.setPosition(.4);
    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); *


    //Parallel: .4889
    //Min Values: .1894
    //Max Values: 1
    //R45 = .3389
    //B45 = .6094
    //AxonRot = .2705/90
     */
        toG1 = follower.pathBuilder()
                .addPath(new BezierCurve(score1, toPickupC1, toPickup1))
                .setLinearHeadingInterpolation(score1.getHeading(), toPickup1.getHeading())
                .build();
        g1 = follower.pathBuilder()
                .addPath(new BezierLine(toPickup1, pickup1))
                .setConstantHeadingInterpolation(pickup1.getHeading())
                .build();
        s1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1, score2))
                .setConstantHeadingInterpolation(score2.getHeading())
                .build();
        toG2 = follower.pathBuilder()
                .addPath(new BezierCurve(score2, toPickupC2, toPickup2))
                .setTangentHeadingInterpolation()
                .build();
        g2 = follower.pathBuilder()
                .addPath(new BezierLine(toPickup2, pickup2))
                .setConstantHeadingInterpolation(pickup2.getHeading())
                .build();
        s2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2, score3))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        toG3 = follower.pathBuilder()
                .addPath(new BezierCurve(score3, toPickupC3, toPickup3))
                .setTangentHeadingInterpolation()
                .build();
        g3 = follower.pathBuilder()
                .addPath(new BezierCurve(toPickup3, pickup3))
                .setConstantHeadingInterpolation(pickup3.getHeading())
                .build();
        s3 = follower.pathBuilder()
                .addPath(new BezierCurve(score1, toPickupC1, toPickup1))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        g4 = follower.pathBuilder()
                .addPath(new BezierCurve(score1, toPickupC1, toPickup1))
                .setTangentHeadingInterpolation()
                .build();
        s4 = follower.pathBuilder()
                .addPath(new BezierCurve(score1, toPickupC1, toPickup1))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    }

    public void autonomousPathUpdate() {
        switch (state) {
            case START:
                flywheelLeft.setVelocity(-1675);
                flywheelRight.setVelocity(-1675);
                hood.setPosition(.67);
                kicker.setPosition(.3);
                follower.setMaxPower(0.9);
                follower.followPath(scorePreload);
                setPathState(State.SCORE);
                actionTimer.resetTimer();

                IntakeOuter.setPower(-.3);
                IntakeInner.setPower(-.3);

                break;
            //After First 3
            case SCORE:
                if(follower.isBusy())
                {
                    actionTimer.resetTimer();
                }
                else if (!(follower.isBusy()))
                {

                    if(actionTimer.getElapsedTimeSeconds() <= 1){




                        // && flywheelRight.getVelocity() > 1650 && flywheelLeft.getVelocity() > 1650 &&
                    } else if (actionTimer.getElapsedTimeSeconds() >= 1 && actionTimer.getElapsedTimeSeconds() <= 3) {

                        IntakeOuter.setPower(-.8);
                        IntakeInner.setPower(-.4);
                        kicker.setPosition(.5);


                    }
                    else if (actionTimer.getElapsedTimeSeconds() >= 3 && actionTimer.getElapsedTimeSeconds() <= 3.1) {
                        kicker.setPosition(.3);


                    }
                    else if(count == 5)
                    {
                        setPathState(State.END);
                    }
                    else {

                            //IntakeInner.setVelocity(0);
                            //IntakeOuter.setVelocity(0);
//                        flywheelLeft.setVelocity(-.01);
//                        flywheelRight.setVelocity(-.01);

                            if (count == 1) {

                                follower.followPath(toG1, true);

                            } else if (count == 2) {
                                follower.followPath(toG2, true);

                            } else if (count == 3) {
                                follower.followPath(toG3);
                            }
                            else if(count == 4)
                            {
                                follower.followPath(g4);
                            }
                            setPathState(State.TRAVEL);
                            count++;

                        }
                    }


                break;


            case TRAVEL:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    if(count == 1)
                    {
                        follower.followPath(g1, true);
                    }
                    if(count == 2)
                    {
                        follower.followPath(g2, true);
                    }
                    if(count == 3)
                    {
                        follower.followPath(g3, true);
                    }
                    if(count == 4)
                    {
                       follower.followPath(s4, true);
                    }

                    setPathState(State.PICKUP);
                    //IntakeInner.setVelocity(300);
                    //IntakeOuter.setVelocity(-300);

                }
                break;
//            case PICKUP2:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
//                if(!follower.isBusy()) {
//                    /* Grab Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(scorePickup2,true);
//                    setPathState(State.PICKUP2RETURN);
//                }
//                break;
//            case 5:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup3,true);
//                    setPathState(6);
//                }
//                break;

            case PICKUP:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    if(count == 1)
                    {
                        follower.followPath(s1, true);
                    }
                    if(count == 2)
                    {
                        follower.followPath(s2, true);
                    }
                    if(count == 3)
                    {
                        follower.followPath(s3, true);
                    }

                    setPathState(State.SCORE);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                }
                break;

            case END:
                if(!follower.isBusy()){


                }
                break;

        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(State stateCooler) {
        state = stateCooler;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", state);
        telemetry.addData("Shooter velocity L", flywheelLeft.getVelocity());
        telemetry.addData("Shooter velocity R", flywheelRight.getVelocity());
        telemetry.addData("Actiontimer",actionTimer.getElapsedTimeSeconds());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("count", count);
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();
        count = 1;

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);



    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(State.START);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}
