
package org.firstinspires.ftc.teamcode.pedroPathing;
import android.net.EthernetNetworkSpecifier;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcontroller.external.samples.SensorMROpticalDistance;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class RedTeleOp extends OpMode {

    private double RED;

    private PathChain toFarShoot;
    private Pose FarShootPose;

    private double GREEN;
    private Follower follower;

    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    private boolean driveState;

    private boolean gateOpen;
    private Servo gate;
    private boolean macroActive;
    private boolean debounceA;
    private Timer pathTimer;

    private  Timer actiontimer;

    private  Timer timerA;

    private Servo raxon;

    private Servo laxon;
    private Servo blocker;
    private Servo hood;

    private double x;
    private double y;

    double ballsPassed;
    private double distance;
    private boolean debounceB;

    private boolean debounceX;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;

    private boolean autoTarget = true;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;

    private DcMotorEx flywheelLeft;

    private DcMotorEx flywheelRight;

    private DcMotorEx intakeOuter;
    private boolean intakeOn;
    private boolean flywheelOn;

    private IMU imu;
    private boolean kickerpos;

    private boolean debounce_dpad_up;
    private boolean debounce_dpad_down;
    private boolean debounceY;

    private boolean debounceDL, debounceDR, debounceLB, debounceRB;

    private  boolean debounceBACK;

    private boolean debounceStart;
    private boolean debounceGUIDE;

    private boolean debounceLEFT_TRIGGER;
    private boolean debounceRIGHT_TRIGGER;

    private double flywheelVelocity;
    private boolean feederOn;
    private DcMotorEx intakeInner;

    private DistanceSensor distanceSensor;
    private CRServo feederL;

    private Servo indicatorLight1;
    private double frontLeftPower;

    double backLeftPower;
    double frontRightPower;
    double backRightPower;

    private Servo indicatorLight2;

    private CRServo feederR;
    private double raxonPos;
    private double laxonPos;
    private double slowModeMultiplier = 0.5;
    private double angleToRot;


    private PathChain parkingSpace, scoringSpot;

    @Override
    public void init() {

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fl");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fr");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "bl");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "br");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(84, 36, Math.toRadians(0)));
        FarShootPose = new Pose(82, 15, Math.toRadians(64.33));
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flyL");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flyR");
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelLeft.setDirection(DcMotor.Direction.FORWARD);
        flywheelRight.setDirection(DcMotor.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");
        gate = hardwareMap.get(Servo.class, "gate");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        indicatorLight1 = hardwareMap.get(Servo.class, "lightOne");
        indicatorLight2 = hardwareMap.get(Servo.class, "lightTwo");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
        intakeOuter = hardwareMap.get(DcMotorEx.class, "intOuter");
        intakeInner = hardwareMap.get(DcMotorEx.class, "intInner");
        intakeOuter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeInner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeOuter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeInner.setDirection(DcMotor.Direction.REVERSE);
        intakeOuter.setDirection(DcMotor.Direction.REVERSE);
        intakeInner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        hood = hardwareMap.get(Servo.class, "hood");
        blocker = hardwareMap.get(Servo.class, "blocker");
        GREEN = .5;
        RED = 0.6;
        flywheelVelocity = 1600;
        gateOpen = false;
        intakeOn = false;
        flywheelOn = false;
        feederOn = false;
        kickerpos = true;
        debounceA = false;
        debounceB = false;
        debounceX = false;
        debounce_dpad_up = false;
        debounce_dpad_down = false;
        debounceY = false;
        debounceBACK = false;
        macroActive = false;
        automatedDrive = false;

        actiontimer = new Timer();

        raxon = hardwareMap.get(Servo.class,"raxon");
        laxon = hardwareMap.get(Servo.class,"laxon");


        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(23.687, 119.835))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(180), 0.8))
                .build();
    }
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)

        blocker.setPosition(.57);
        indicatorLight1.setPosition(RED);
        indicatorLight2.setPosition(RED);
        gate.setPosition(.55);
        follower.startTeleopDrive();
        follower.setMaxPower(1);
        kickerpos = false;
        raxon.setPosition(.48);
        laxon.setPosition(.48);
        hood.setPosition(.5694);
        imu.resetYaw();
        //Parallel: .5
        //Min Values: .1
        //Max Values: 1
        //R45 = .36
        //B45 = .64
        //AxonRot CCW = .28/90

    }
    @Override
    public void loop() {




        if(autoTarget)
        {

            x = follower.getPose().getX();
            y = follower.getPose().getY();
            distance = Math.sqrt(Math.pow(144-y,2) + Math.pow(144-x,2));
            flywheelVelocity = 8.87 * (distance) + 1000;
            hood.setPosition((-.00554324 * distance + .96));


//            angleToRot = (imu.getRobotYawPitchRollAngles().getYaw()) - Math.toDegrees(Math.atan((138-y)/(138-x)));
//            laxonPos = .5 + (.28/90)*angleToRot; //if not work subtract laxon and act raxon
//            raxonPos = .5 + (.28/90)*angleToRot;
//            raxon.setPosition(raxonPos);
//            laxon.setPosition(laxonPos);



        }








        //
        //raxonPos = (.5+(.43/180)*(Math.toDegrees(Math.atan((144-y)/(144-x)))) + (.43/180)*(imu.getRobotYawPitchRollAngles().getYaw()));
        //laxonPos = (.5-(.57/180)*(Math.toDegrees(Math.atan((144-y)/(144-x)))) - (.57/180)*(imu.getRobotYawPitchRollAngles().getYaw()));

        distance = Math.sqrt(Math.pow(144-y,2) + Math.pow(144-x,2));



        //flywheelVelocity = .0701544 * Math.pow(distance,2) - 3.07502 * distance + 1200;
        //hood.setPosition(.259228 * Math.sin(.03483 * distance + .48236) + .752718);


        if(raxonPos > 1)
        {
            raxonPos = 1;
        }
        if(raxonPos < .1)
       {
            raxonPos = .1;
        }
        if(laxonPos < .1)
        {
            laxonPos = .1;
        }
        if(laxonPos > 1)
        {
            laxonPos = 1;
        }

        //raxon.setPosition(raxonPos);
        //laxon.setPosition(laxonPos);


        if (gamepad1.back && debounceBACK && kickerpos){
            kickerpos = false;
            gate.setPosition(.55);
            debounceBACK = false;
            indicatorLight1.setPosition(RED);
            indicatorLight2.setPosition(RED);

        }
        if (gamepad1.back && debounceBACK && !kickerpos){
            gate.setPosition(.88);
            kickerpos = true;
            debounceBACK = false;
            indicatorLight1.setPosition(GREEN);
            indicatorLight2.setPosition(GREEN);
            actiontimer.resetTimer();
        }


        if(!gamepad1.back){

            debounceBACK = true;
        }

        if (gamepad1.a && !intakeOn && !debounceA){
            debounceA = true;
            intakeOn = true;

        }
        if (intakeOn) {
            intakeOuter.setPower(-.8);

            if (kickerpos){
                intakeInner.setPower(.4);
            }
            else{
                intakeInner.setPower(0);
            }

        }

        if (!intakeOn) {


            intakeOuter.setPower(0);
            intakeInner.setPower(0);
        }
        if (gamepad1.a && intakeOn && !debounceA){
            debounceA = true;
            intakeOn = false;

        }
        if (gamepad1.b && !feederOn && !debounceB){
            debounceB = true;
            feederOn = true;
            intakeOuter.setVelocity(900);
            intakeInner.setVelocity(-900);
//            feederL.setPower(-1);
//            feederR.setPower(1);
        }

        if (gamepad1.left_bumper && debounceLB){
            hood.setPosition(hood.getPosition()-.05);
            debounceLB = false;
        }
        if(!gamepad1.left_bumper)
        {
            debounceLB = true;
        }
        if (gamepad1.right_bumper && debounceRB){
            hood.setPosition(hood.getPosition()+.05);
            debounceRB = false;
        }
        if(!gamepad1.right_bumper)
        {
            debounceRB = true;
        }

        if (gamepad1.guide && debounceGUIDE){
            automatedDrive = true;

                toFarShoot = follower.pathBuilder()
                        .addPath(new Path(new BezierLine(follower::getPose, FarShootPose)))

                        .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, FarShootPose.getHeading(), 0.8))

                        .setTValueConstraint(.8)

                        .build();

                follower.followPath(toFarShoot);

                debounceGUIDE = false;
        }





        if (!gamepad1.guide){
            debounceGUIDE = true;
        }





        if (gamepad1.b && feederOn && !debounceB){
            debounceB = true;
            feederOn = false;
            intakeOuter.setVelocity(0);
            intakeInner.setVelocity(0);
//            feederL.setPower(0);
//            feederR.setPower(0);
        }

        if (gamepad1.x && !flywheelOn && !debounceX){
            debounceX = true;
            flywheelOn = true;
        }
        if (gamepad1.x && flywheelOn && !debounceX){
            debounceX = true;
            flywheelOn = false;
        }
        if(flywheelOn){
            flywheelLeft.setVelocity(flywheelVelocity);
            flywheelRight.setVelocity(flywheelVelocity);
        }
        if(!flywheelOn){
            flywheelLeft.setVelocity(-.01);
            flywheelRight.setVelocity(-.01);
        }

        if (gamepad1.left_trigger > .01 && debounceLEFT_TRIGGER){
            raxonPos = raxon.getPosition() -.02;
            laxonPos = laxon.getPosition() - .02;
            laxon.setPosition(laxonPos);
            raxon.setPosition(raxonPos);

            debounceLEFT_TRIGGER = false;
        }
        if (gamepad1.right_trigger > .01 && debounceRIGHT_TRIGGER){
            raxonPos = raxon.getPosition() + .02;
            laxonPos = laxon.getPosition() + .02;
            raxon.setPosition(raxonPos);
            laxon.setPosition(laxonPos);
            debounceRIGHT_TRIGGER = false;

        }




        if (gamepad1.left_trigger < .01){
            debounceLEFT_TRIGGER = true;
        }
        if (gamepad1.right_trigger < .01){
            debounceRIGHT_TRIGGER = true;
        }




        if(!gamepad1.a){
            debounceA = false;
        }
        if(!gamepad1.b){
            debounceB = false;
        }
        if(!gamepad1.x){
            debounceX = false;
        }

        if(gamepad1.dpad_left && debounceDL)
        {
            gate.setPosition(gate.getPosition() + .03);
        }
        if(gamepad1.dpad_right && debounceDR)
        {
            gate.setPosition(gate.getPosition() - .03);
        }
        if(!gamepad1.dpad_left)
        {
            debounceDL = true;
        }
        if(!gamepad1.dpad_right)
        {
            debounceDR = true;
        }
        //Call this once per loop
        follower.update();
        telemetryM.update();


        if(gamepad1.dpad_up && flywheelOn && !debounce_dpad_up){
            flywheelVelocity += 50;
            flywheelLeft.setVelocity(flywheelVelocity);
            flywheelRight.setVelocity(flywheelVelocity);
            debounce_dpad_up = true;
        }
        if(gamepad1.dpad_down && flywheelOn && !debounce_dpad_down){
            flywheelVelocity -= 50;
            flywheelLeft.setVelocity(flywheelVelocity);
            flywheelRight.setVelocity(flywheelVelocity);
            debounce_dpad_down = true;
        }

        if(!gamepad1.dpad_up){
            debounce_dpad_up = false;
        }
        if(!gamepad1.dpad_down){
            debounce_dpad_down = false;
        }
        if(gamepad1.y && debounceY)
        {
            autoTarget = !autoTarget;
            laxonPos = .5;
            raxonPos = .5;
            debounceY = false;
        }
        if(!gamepad1.y)
        {
            debounceY = true;
        }

        if(gamepad1.start && debounceStart){
            macroActive = true;
            actiontimer.resetTimer();

        }






        if(!gamepad1.start){
            debounceStart = true;
        }





        /*   if (!automatedDrive) {
            if (follower.getPose().getY() > 72) {
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.right_stick_x * .3;

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
            }


            //This is how it looks with slowMode on
            else {
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.right_stick_x * .5;

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
            }

        }


      */

        if (!automatedDrive) {
            if (follower.getPose().getY() > 72) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        true // Robot Centric
                );
            }


            //This is how it looks with slowMode on
            else {
                    follower.setTeleOpDrive(
                            -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x * .3,
                    true // Robot Centric
            );

            }

        }
            // 2/1 Mr. B Commented out the following if statement to test
            if (((gamepad1.guide && debounceGUIDE) || !follower.isBusy()) && automatedDrive) {
                debounceGUIDE = false;
                automatedDrive = false;
                follower.startTeleopDrive();
            }


            //}   2/1 Mr. B moved this to after the Telemetry (to mirror BlueTeleop)

            //Automated PathFollowing

            //Stop automated following if the follower is done


            //Slow Mode


            //Optional way to change slow mode strength


            //Optional way to change slow mode strength

            telemetry.addData("FL Power", frontLeftMotor.getPower());
            telemetry.addData("BL Power", backLeftMotor.getPower());
            telemetry.addData("FR Power", frontRightMotor.getPower());
            telemetry.addData("BR Power", backRightMotor.getPower());
//            telemetry.addData("axonL", laxon.getPosition());
//            telemetry.addData("axonR", raxon.getPosition());
//            telemetry.addData("blocker pos", blocker.getPosition());
//            telemetry.addData("Hood position", hood.getPosition());
//            //telemetry.addData("raxon",raxon.getPosition());
//            //telemetry.addData("laxon",laxon.getPosition());
//            telemetry.addData("atr", angleToRot);
//            telemetry.addData("flywheel velocity", flywheelLeft.getVelocity());
//            telemetry.addData("debounce y", debounceY);
//            telemetry.addData("Angle(y/x)", Math.toDegrees(Math.atan((144 - y) / (144 - x))));
//            telemetry.addData("Angle(x/y)", Math.toDegrees(Math.atan((144 - x) / (144 - y))));
//            telemetry.addData("position", follower.getPose());
//            /*telemetryM.debug("position", follower.getPose()); */
//
//            telemetry.addData("YAW", imu.getRobotYawPitchRollAngles().getYaw());
//            telemetry.addData("distance", distance);
//            telemetry.addData("Distance Sensor", distanceSensor.getDistance(DistanceUnit.CM));
//            telemetry.addData("gate", gate.getPosition());
//            telemetry.addData("balls shot this burst", ballsPassed);
//            telemetry.addData("heading according to pedro", follower.getHeading());


        telemetry.addData("axonL", laxon.getPosition());
        telemetry.addData("axonR", raxon.getPosition());
        telemetry.addData("blocker pos",blocker.getPosition());
        telemetry.addData("Hood position", hood.getPosition());
        //telemetry.addData("raxon",raxon.getPosition());
        //telemetry.addData("laxon",laxon.getPosition());
        telemetry.addData("atr", angleToRot);
        telemetry.addData("flywheel velocity",flywheelLeft.getVelocity());
        telemetry.addData("attempted flywheel velocity", 8.87 * (distance) + 1000);
        telemetry.addData("debounce y", debounceY);
        telemetry.addData("Pedro pose", follower.getPose());

        telemetry.addData("heading", follower.getHeading());
        /*telemetryM.debug("position", follower.getPose()); */
        telemetry.addData("FL motor", frontLeftMotor.getVelocity());
        telemetry.addData("FR motor", frontRightMotor.getVelocity());
        telemetry.addData("BR motor", backRightMotor.getVelocity());
        telemetry.addData("BL motor", backLeftMotor.getVelocity());
        telemetry.addData("YAW", imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.addData("distance", distance);
        telemetry.addData("Distance Sensor", distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("gate", gate.getPosition() );
        telemetry.addData("balls shot this burst" ,ballsPassed);
        telemetry.addData("heading according to pedro" , follower.getHeading());
        }

        //Automated PathFollowing

        //Stop automated following if the follower is done


        //Slow Mode


        //Optional way to change slow mode strength


        //Optional way to change slow mode strength



    }


