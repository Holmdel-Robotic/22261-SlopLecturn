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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
public class BlueTeleOpLL extends OpMode {

    private double RED;
    private double defaultVelo;
    private double defaultHood;
    private double GREEN;
    private Follower follower;

    private boolean driveState;
    private Servo gate;
    private boolean macroActive;
    private boolean debounceA;
    private Timer pathTimer;

    private Timer actiontimer;

    private Timer timerA;

    private Servo raxon;

    private Servo laxon;
    private Servo blocker;
    private Servo hood;

    private double x;
    private double y;

    double ballsPassed;
    private double distance;
    private boolean debounceB;

    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
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

    private boolean debounceBACK;

    private boolean debounceStart;
    private boolean debounceGUIDE;

    private boolean debounceLEFT_TRIGGER;
    private boolean debounceRIGHT_TRIGGER;

    private double flywheelVelocity;
    private boolean feederOn;
    private DcMotorEx intakeInner;

    private DistanceSensor distanceSensor;
    private CRServo feederL;

    private double BLUE;
    private Servo indicatorLight1;

    private Servo indicatorLight2;

    private CRServo feederR;
    private double raxonPos;
    private double laxonPos;
    private double slowModeMultiplier = 0.5;
    private double angleToRot;

    // AprilTag tracking variables
    private Limelight3A limelight;
    private AnalogInput encoder;
    private double kP = 0.09;
    private double max = 0.02;
    private boolean aprilTagTracking = false;
    private boolean debounceRightStick;

    private PathChain parkingSpace, scoringSpot;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fl");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fr");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "bl");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "br");
        defaultHood = .64;
        defaultVelo = 1540;
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(144 - 84, 36, Math.toRadians(180)));
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
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

        // Initialize AprilTag tracking hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        encoder = hardwareMap.get(AnalogInput.class, "absEncoder");
        limelight.pipelineSwitch(1);
        limelight.start();

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
        BLUE = .6;
        flywheelVelocity = 1600;
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
        debounceRightStick = false;


        actiontimer = new Timer();

        raxon = hardwareMap.get(Servo.class, "raxon");
        laxon = hardwareMap.get(Servo.class, "laxon");


        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(23.687, 119.835))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(180), 0.8))
                .build();
    }

    @Override
    public void start() {
        gate.setPosition(.5);
        follower.startTeleopDrive();
        follower.setMaxPower(.8);
        blocker.setPosition(.3);
        indicatorLight1.setPosition(BLUE);
        indicatorLight2.setPosition(BLUE);
        kickerpos = false;
        raxon.setPosition(.48);
        laxon.setPosition(.48);
        raxonPos = .48;
        laxonPos = .48;
//        hood.setPosition(.8);
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

        // Toggle AprilTag tracking with right stick (press down on right stick)
        if (gamepad1.right_stick_button && debounceRightStick) {
            aprilTagTracking = !aprilTagTracking;
            debounceRightStick = false;
        }
        if (!gamepad1.right_stick_button) {
            debounceRightStick = true;
        }

        // LL runs continuously when enabled
        if (aprilTagTracking) {
            trackAprilTag();
        }

        if (autoTarget) {

            x = follower.getPose().getX();
            y = follower.getPose().getY();
            distance = Math.sqrt(Math.pow(144 - y, 2) + Math.pow(x, 2));
            flywheelVelocity = 8.87 * (distance) + 1000;
            hood.setPosition((-.00554324 * distance + .95));

        }

        if (raxonPos > 1) {
            raxonPos = 1;
        }
        if (raxonPos < .1) {
            raxonPos = .1;
        }
        if (laxonPos < .1) {
            laxonPos = .1;
        }
        if (laxonPos > 1) {
            laxonPos = 1;
        }

        // Only manual servo control if LL is OFF
        if (!aprilTagTracking) {
            raxon.setPosition(raxonPos);
            laxon.setPosition(laxonPos);
        }


        if (gamepad1.back && debounceBACK && kickerpos){
            kickerpos = false;
            blocker.setPosition(.3);
            debounceBACK = false;
            indicatorLight1.setPosition(RED);
            indicatorLight2.setPosition(RED);

        }
        if (gamepad1.back && debounceBACK && !kickerpos){
            blocker.setPosition(.57 );
            kickerpos = true;
            debounceBACK = false;
            indicatorLight1.setPosition(GREEN);
            indicatorLight2.setPosition(GREEN);
            actiontimer.resetTimer();
        }

        if (kickerpos){
            indicatorLight1.setPosition(GREEN);
            indicatorLight2.setPosition(GREEN);
        }else{
            indicatorLight1.setPosition(BLUE);
            indicatorLight2.setPosition(BLUE);
        }

        if (!gamepad1.back) {

            debounceBACK = true;
        }

        if (gamepad1.a && !intakeOn && !debounceA) {
            debounceA = true;
            intakeOn = true;

        }
        if (intakeOn) {
            intakeOuter.setPower(-.8);

            if (distanceSensor.getDistance(DistanceUnit.CM) > 13.5 || kickerpos) {
                intakeInner.setPower(.4);
            } else {
                intakeInner.setPower(0);
            }

        }

        if (!intakeOn) {


            intakeOuter.setPower(0);
            intakeInner.setPower(0);
        }
        if (gamepad1.a && intakeOn && !debounceA) {
            debounceA = true;
            intakeOn = false;

        }
        if (gamepad1.b && !feederOn && !debounceB) {
            debounceB = true;
            feederOn = true;
            intakeOuter.setVelocity(900);
            intakeInner.setVelocity(-900);
//            feederL.setPower(-1);
//            feederR.setPower(1);
        }

        if (gamepad1.left_bumper && debounceLB) {
            hood.setPosition(hood.getPosition() - .05);
            debounceLB = false;
        }
        if (!gamepad1.left_bumper) {
            debounceLB = true;
        }
        if (gamepad1.right_bumper && debounceRB) {
            hood.setPosition(hood.getPosition() + .05);
            debounceRB = false;
        }
        if (!gamepad1.right_bumper) {
            debounceRB = true;
        }

        if (gamepad1.guide) {


            //

            follower.turnTo(Math.toRadians(135));
            debounceGUIDE = false;
            driveState = false;

        }
        if (!gamepad1.guide && !driveState) {
            follower.startTeleopDrive();
            driveState = true;
            debounceGUIDE = false;
        }


        if (!gamepad1.guide) {
            debounceGUIDE = true;
        }


        if (gamepad1.b && feederOn && !debounceB) {
            debounceB = true;
            feederOn = false;
            intakeOuter.setVelocity(0);
            intakeInner.setVelocity(0);
//            feederL.setPower(0);
//            feederR.setPower(0);
        }

        if (gamepad1.x && !flywheelOn && !debounceX) {
            debounceX = true;
            flywheelOn = true;
        }
        if (gamepad1.x && flywheelOn && !debounceX) {
            debounceX = true;
            flywheelOn = false;
        }
        if (flywheelOn) {
            flywheelLeft.setVelocity(flywheelVelocity);
            flywheelRight.setVelocity(flywheelVelocity);
        }
        if (!flywheelOn) {
            flywheelLeft.setVelocity(-.01);
            flywheelRight.setVelocity(-.01);
        }

        // Only manual trigger if LL is OFF
        if (!aprilTagTracking) {
            if (gamepad1.left_trigger > .01 && debounceLEFT_TRIGGER) {
                raxonPos = raxon.getPosition() + .03;
                laxonPos = laxon.getPosition() - .03;
                laxon.setPosition(laxonPos);
                raxon.setPosition(raxonPos);

                debounceLEFT_TRIGGER = false;
            }
            if (gamepad1.right_trigger > .01 && debounceRIGHT_TRIGGER) {
                raxonPos = raxon.getPosition() - .03;
                laxonPos = laxon.getPosition() + .03;
                raxon.setPosition(raxonPos);
                laxon.setPosition(laxonPos);
                debounceRIGHT_TRIGGER = false;

            }
        }


        if (gamepad1.left_trigger < .01) {
            debounceLEFT_TRIGGER = true;
        }
        if (gamepad1.right_trigger < .01) {
            debounceRIGHT_TRIGGER = true;
        }


        if (!gamepad1.a) {
            debounceA = false;
        }
        if (!gamepad1.b) {
            debounceB = false;
        }
        if (!gamepad1.x) {
            debounceX = false;
        }

        if (gamepad1.dpad_left && debounceDL) {
            gate.setPosition(gate.getPosition() + .03);
        }
        if (gamepad1.dpad_right && debounceDR) {
            gate.setPosition(gate.getPosition() - .03);
        }
        if (!gamepad1.dpad_left) {
            debounceDL = true;
        }
        if (!gamepad1.dpad_right) {
            debounceDR = true;
        }
        follower.update();
        telemetryM.update();


        if (gamepad1.dpad_up && flywheelOn && !debounce_dpad_up) {
            flywheelVelocity += 200;
            flywheelLeft.setVelocity(flywheelVelocity);
            flywheelRight.setVelocity(flywheelVelocity);
            debounce_dpad_up = true;
        }
        if (gamepad1.dpad_down && flywheelOn && !debounce_dpad_down) {
            flywheelVelocity -= 200;
            flywheelLeft.setVelocity(flywheelVelocity);
            flywheelRight.setVelocity(flywheelVelocity);
            debounce_dpad_down = true;
        }

        if (!gamepad1.dpad_up) {
            debounce_dpad_up = false;
        }
        if (!gamepad1.dpad_down) {
            debounce_dpad_down = false;
        }
        if (gamepad1.y && debounceY) {
            autoTarget = !autoTarget;
            laxonPos = .5;
            raxonPos = .5;
            flywheelVelocity = defaultVelo;
            hood.setPosition(defaultHood);
            debounceY = false;
        }
        if (!gamepad1.y) {
            debounceY = true;
        }

        if (gamepad1.start && debounceStart) {
            macroActive = true;
            actiontimer.resetTimer();

        }


        if (actiontimer.getElapsedTime() < 3000 && macroActive) {


            indicatorLight1.setPosition(GREEN);
            blocker.setPosition(.50);
            intakeOuter.setVelocity(-800);
            intakeInner.setVelocity(400);
        } else if (actiontimer.getElapsedTime() > 3000 && macroActive) {
            indicatorLight1.setPosition(BLUE);
            blocker.setPosition(.3);
            intakeOuter.setVelocity(-.01);
            intakeInner.setPower(-.01);
            macroActive = false;
        }

        if (!gamepad1.start) {
            debounceStart = true;
        }


        if (!automatedDrive) {

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

            telemetry.addData("AprilTag Tracking", aprilTagTracking ? "ENABLED" : "DISABLED");
            telemetry.addData("axonL", laxon.getPosition());
            telemetry.addData("axonR", raxon.getPosition());
            telemetry.addData("blocker pos", blocker.getPosition());
            telemetry.addData("Hood position", hood.getPosition());
            //telemetry.addData("raxon",raxon.getPosition());
            //telemetry.addData("laxon",laxon.getPosition());
            telemetry.addData("atr", angleToRot);
            telemetry.addData("flywheel velocity", flywheelLeft.getVelocity());
            telemetry.addData("debounce y", debounceY);

            telemetry.addData("position", follower.getPose());
            /*telemetryM.debug("position", follower.getPose()); */
            //telemetryM.debug("velocity", follower.getVelocity());
            //telemetryM.debug("automatedDrive", automatedDrive);
            telemetry.addData("YAW", imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.addData("distance", distance);
            telemetry.addData("Distance Sensor", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("gate", gate.getPosition());
            telemetry.addData("balls shot this burst", ballsPassed);
            telemetry.addData("heading according to pedro", follower.getHeading());
            telemetry.addData("Encoder", "%.0f°", (encoder.getVoltage() / 3.3) * 360);

        }
    }

    // AprilTag tracking method
    private void trackAprilTag() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            double error = result.getFiducialResults().get(0).getTargetXDegrees();

            if (Math.abs(error) > 2) {
                // Calculate correction
                double correction = kP * error;
                correction = Math.max(-max, Math.min(max, correction));

                // Update position
                raxonPos += correction;
                laxonPos += correction;

                // Clamp positions
                raxonPos = Math.max(0.1, Math.min(1.0, raxonPos));
                laxonPos = Math.max(0.1, Math.min(1.0, laxonPos));

                // Set servos
                laxon.setPosition(laxonPos);
                raxon.setPosition(raxonPos);

                telemetry.addData("AT Target ID", result.getFiducialResults().get(0).getFiducialId());
                telemetry.addData("AT Error", "%.2f°", error);
                telemetry.addData("AT Correction", "%.5f", correction);
                telemetry.addData("AT Status", "TRACKING");
            } else {
                telemetry.addData("AT Status", "✓ LOCKED ON");
                telemetry.addData("AT Error", "%.2f°", error);
            }
        } else {
            telemetry.addData("AT Status", "⚠ Searching...");
        }
    }

}