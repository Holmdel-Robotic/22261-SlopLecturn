package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

@Config
@TeleOp
public class pleaseworkBlueTeleopLL extends OpMode {

    private Follower follower;

    private VoltageSensor myControlHubVoltageSensor;

    private Pose pose;

    private Servo gate, indicatorLight1, indicatorLight2;
    private Servo hood, raxon, laxon;

    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    private DcMotorEx flywheelLeft, flywheelRight, intakeOuter, intakeInner;

    private DistanceSensor intakeSensor1, intakeSensor2, innerSensor;

    private boolean gateOpen = false;
    private boolean driving;

    public static boolean intakeOn = false;
    public static boolean flywheelOn = false;
    public static boolean aprilTagTracking = false;
    public static boolean autoAim;

    private boolean debounceA, debounceX, debounceBack, debounceRightStick;
    private boolean debounceDpad_up, debounceDpad_down, debounceDpad_left, debounceDpad_right;

    public static double flywheelVelocity = 800;
    public static double hoodPos;
    public static double axonPos = .48;

    private double intake1Dist, intake2Dist, innerSensorDist;
    private boolean intakeFull;

    private double lastUpdateTime;
    private double savedRuntime;

    private double GREEN = .5;
    private double BLUE = .6;

    private long lastLoopTime;

    @Override
    public void init() {

        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        gate = hardwareMap.get(Servo.class, "gate");
        indicatorLight1 = hardwareMap.get(Servo.class, "lightOne");
        indicatorLight2 = hardwareMap.get(Servo.class, "lightTwo");

        hood = hardwareMap.get(Servo.class, "hood");
        raxon = hardwareMap.get(Servo.class, "raxon");
        laxon = hardwareMap.get(Servo.class, "laxon");

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fl");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fr");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "bl");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "br");

        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flyL");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flyR");

        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheelRight.setDirection(DcMotor.Direction.REVERSE);

        intakeOuter = hardwareMap.get(DcMotorEx.class, "intOuter");
        intakeInner = hardwareMap.get(DcMotorEx.class, "intInner");

        intakeOuter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeInner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeOuter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeInner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeInner.setDirection(DcMotor.Direction.REVERSE);
        intakeOuter.setDirection(DcMotor.Direction.REVERSE);

        intakeSensor1 = hardwareMap.get(DistanceSensor.class, "intakeSensor1");
        intakeSensor2 = hardwareMap.get(DistanceSensor.class, "intakeSensor2");
        innerSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(60, 36, Math.toRadians(180)));

        indicatorLight1.setPosition(BLUE);
        indicatorLight2.setPosition(BLUE);

        raxon.setPosition(axonPos);
        laxon.setPosition(axonPos);

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        lastLoopTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {

        processGamepad1();
        updateRobotState();
        writeHardware();
        driveRobot();
        follower.update();
        updateTelemetry();
    }

    private void processGamepad1() {

        if (gamepad1.back && !debounceBack) {
            savedRuntime = getRuntime();
            gateOpen = !gateOpen;
            debounceBack = true;
        }
        if (!gamepad1.back) debounceBack = false;

        if (gamepad1.a && !debounceA) {
            intakeOn = !intakeOn;
            debounceA = true;
        }
        if (!gamepad1.a) debounceA = false;

        if (gamepad1.x && !debounceX) {
            flywheelOn = !flywheelOn;
            debounceX = true;
        }
        if (!gamepad1.x) debounceX = false;

        if (gamepad1.right_stick_button && !debounceRightStick) {
            aprilTagTracking = !aprilTagTracking;
            debounceRightStick = true;
        }
        if (!gamepad1.right_stick_button) debounceRightStick = false;
    }

    private void updateRobotState() {

        intake1Dist = intakeSensor1.getDistance(DistanceUnit.CM);
        intake2Dist = intakeSensor2.getDistance(DistanceUnit.CM);
        innerSensorDist = innerSensor.getDistance(DistanceUnit.CM);

        intakeFull = intake1Dist < 10 && intake2Dist < 10;

        if (getRuntime() - lastUpdateTime > .05 && autoAim) {
            lastUpdateTime = getRuntime();
            pose = follower.getPose();
            double x = pose.getX();
            double y = pose.getY();
            double dy = 144 - y;
            double distance = Math.sqrt(dy * dy + x * x);
            hoodPos = -0.004005998 * distance + 1;
        }
    }

    private void writeHardware() {

        hood.setPosition(hoodPos);

        if (gateOpen && getRuntime() - savedRuntime < 2) {

            flywheelOn = true;
            gate.setPosition(.88);

            intakeInner.setPower(.65);
            intakeOuter.setPower(-.65);

            indicatorLight1.setPosition(GREEN);
            indicatorLight2.setPosition(GREEN);

        } else {

            gate.setPosition(.5);
            indicatorLight1.setPosition(BLUE);
            indicatorLight2.setPosition(BLUE);

            intakeInner.setPower(.65);
        }

        if (intakeOn && !intakeFull && !gateOpen) {
            intakeOuter.setPower(-.8);
        } else if (intakeOn && intakeFull && !gateOpen) {
            intakeOuter.setPower(0);
        } else if (!intakeOn) {
            intakeOuter.setPower(0);
        }

        if (flywheelOn) {
            flywheelLeft.setVelocity(flywheelVelocity);
            flywheelRight.setVelocity(flywheelVelocity);
        } else {
            flywheelLeft.setVelocity(0);
            flywheelRight.setVelocity(0);
        }

        raxon.setPosition(axonPos);
        laxon.setPosition(axonPos);
    }

    private void driveRobot() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x * .4;

        driving = !(Math.abs(y) < .03 && Math.abs(x) < .03 && Math.abs(rx) < .03);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        frontLeftMotor.setPower((y + x + rx) / denominator);
        backLeftMotor.setPower((y - x + rx) / denominator);
        frontRightMotor.setPower((y - x - rx) / denominator);
        backRightMotor.setPower((y + x - rx) / denominator);
    }

    private void updateTelemetry() {

        long now = System.currentTimeMillis();
        long loopTime = now - lastLoopTime;
        lastLoopTime = now;

        telemetry.addData("FlywheelOn", flywheelOn);
        telemetry.addData("IntakeOn", intakeOn);
        telemetry.addData("GateOpen", gateOpen);
        telemetry.addData("FlywheelV L", flywheelLeft.getVelocity());
        telemetry.addData("FlywheelV R", flywheelRight.getVelocity());
        telemetry.addData("Intake1", intake1Dist);
        telemetry.addData("Intake2", intake2Dist);
        telemetry.addData("Inner", innerSensorDist);
        telemetry.addData("Loop ms", loopTime);
        telemetry.addData("Hz", 1000.0 / loopTime);
        telemetry.update();
    }
}