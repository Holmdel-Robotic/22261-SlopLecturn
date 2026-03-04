package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;

@Config
@TeleOp
public class ChatBlueTeleopTest extends OpMode {

    private Follower follower;

    public double testMillis, lastTestMillis = getRuntime();

    private double processgamepadMillis;
    private double updateRobotStateMillis;
    private double writeHardwareMillis;
    private VoltageSensor myControlHubVoltageSensor;
    private double driveRobotMillis;
    private double lastUpdateTime = getRuntime(), lastProcessgamepadMillis = getRuntime(),
            lastUpdateRobotStateMillis = getRuntime(), lastWriteHardwareMillis = getRuntime(),
            lastDriveRobotMillis = getRuntime();

    private Pose pose;

    double innerSensorDist;
    double intake1Dist;
    double intake2Dist;
    boolean intakeFull;
    public static boolean autoAim;

    private Servo gate, indicatorLight1, indicatorLight2;
    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    private DcMotorEx flywheelLeft, flywheelRight, intakeOuter, intakeInner;
    private DistanceSensor intakeSensor1, intakeSensor2, innerSensor;
    private Servo hood, raxon, laxon;

    private boolean gateOpen = false;
    private boolean debounceDistance = false;
    private boolean driving;

    public static boolean intakeOn = false;
    public static boolean flywheelOn = false;
    public static boolean aprilTagTracking = false;

    private boolean debounceA, debounceX, debounceRightStick, debounceBack,
            debounceLEFT_TRIGGER, debounceY, debounceDpad_up, debounceDpad_down,
            debounceDpad_left, debbounceDpad_right;

    public static double flywheelVelocity = 800;
    public static double hoodPos;
    public static double raxonPos = .48;
    public static double laxonPos = .48;
    public static double axonPos = .48;
    private double currentFlywheelVelocity = 0;
    private static final double FLYWHEEL_RAMP_STEP = 50; // ticks/sec per loop (idk if this is right)

    private int loopCount = 0;
    private long lastLoopTime;
    private double savedRuntime;

    private double kP = 0.843;
    private double max = 0.00922;

    private double GREEN = .5;
    private double BLUE = .6;
    private double distance;

    @Override
    public void init() {

        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        gate = hardwareMap.get(Servo.class, "gate");
        indicatorLight1 = hardwareMap.get(Servo.class, "lightOne");
        indicatorLight2 = hardwareMap.get(Servo.class, "lightTwo");

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fl");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fr");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "bl");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "br");
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flyL");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flyR");
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelLeft.setDirection(DcMotor.Direction.FORWARD);
        flywheelRight.setDirection(DcMotor.Direction.REVERSE);

        intakeOuter = hardwareMap.get(DcMotorEx.class, "intOuter");
        intakeInner = hardwareMap.get(DcMotorEx.class, "intInner");
        intakeOuter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeInner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeOuter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeInner.setDirection(DcMotor.Direction.REVERSE);
        intakeOuter.setDirection(DcMotor.Direction.REVERSE);
        intakeInner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeSensor1 = hardwareMap.get(DistanceSensor.class, "intakeSensor1");
        intakeSensor2 = hardwareMap.get(DistanceSensor.class, "intakeSensor2");
        innerSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        hood = hardwareMap.get(Servo.class, "hood");
        raxon = hardwareMap.get(Servo.class, "raxon");
        laxon = hardwareMap.get(Servo.class, "laxon");

        autoAim = false;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(144 - 84, 36, Math.toRadians(180)));

        lastLoopTime = System.currentTimeMillis();

        indicatorLight1.setPosition(BLUE);
        indicatorLight2.setPosition(BLUE);

        raxon.setPosition(.48);
        laxon.setPosition(.48);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void loop() {
        loopCount++;
        processGamepad1();
        updateRobotState();
        writeHardware();
        driveRobot();
        follower.update();
        updateTelemetry();
    }

    /* ================= INPUT PROCESSING ================= */

    private void processGamepad1() {

        if (gamepad1.dpad_up && !debounceDpad_up) {
            flywheelVelocity += 50;
            debounceDpad_up = true;
        }
        if (gamepad1.dpad_down && !debounceDpad_down) {
            flywheelVelocity -= 50;
            debounceDpad_down = true;
        }
        if (gamepad1.dpad_left && !autoAim && !debounceDpad_left) {
            hoodPos += .02;
            debounceDpad_left = true;
        }
        if (gamepad1.dpad_right && !autoAim && !debbounceDpad_right) {
            hoodPos -= .02;
            debbounceDpad_right = true;
        }

        if (!gamepad1.dpad_up)    debounceDpad_up = false;
        if (!gamepad1.dpad_down)  debounceDpad_down = false;
        if (!gamepad1.dpad_left)  debounceDpad_left = false;
        if (!gamepad1.dpad_right) debbounceDpad_right = false;

        if (gamepad1.y && !debounceY) {
            autoAim = !autoAim;
            flywheelVelocity = 800;
            debounceY = true;
        }
        if (!gamepad1.y) debounceY = false;

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

        if (gamepad1.right_stick_button && debounceRightStick) {
            aprilTagTracking = !aprilTagTracking;
            debounceRightStick = false;
        }
        if (!gamepad1.right_stick_button) debounceRightStick = true;

        // there was two writes for raxon and laxon that was also at bottom of writeHardware()
        if (!aprilTagTracking) {
            raxon.setPosition(axonPos);
            laxon.setPosition(axonPos);
        }

        if (!aprilTagTracking && gamepad1.left_trigger > .01 && debounceLEFT_TRIGGER) {
            raxonPos = raxon.getPosition() + .03;
            laxonPos = laxon.getPosition() + .03;
            laxon.setPosition(laxonPos);
            raxon.setPosition(raxonPos);
            debounceLEFT_TRIGGER = false;
        }

        if (aprilTagTracking) {
            trackAprilTag();
        }
    }

    /* ================= ROBOT LOGIC ================= */

    private void updateRobotState() {
        lastTestMillis = getRuntime();
        intake1Dist = intakeSensor1.getDistance(DistanceUnit.CM);
        intake2Dist = intakeSensor2.getDistance(DistanceUnit.CM);
        intakeFull = intake1Dist < 10 && intake2Dist < 10;
        innerSensorDist = innerSensor.getDistance(DistanceUnit.CM);

        if (getRuntime() - lastUpdateTime > .05 && autoAim) {
            lastUpdateTime = getRuntime();
            pose = follower.getPose();
            double x = pose.getX();
            double y = pose.getY();
            double dy = 144 - y;
            distance = Math.sqrt(dy * dy + x * x);
            hoodPos = -0.004005998 * distance + 1.0;
        }

        testMillis = getRuntime() - lastTestMillis;
    }

    /* ================= HARDWARE WRITES ================= */

    private void writeHardware() {

        hood.setPosition(hoodPos);

        if (driving && autoAim) {
            raxon.setPosition(axonPos);
            laxon.setPosition(axonPos);
        }

        if (gateOpen && getRuntime() - savedRuntime < 2) {
            flywheelOn = true;
            gate.setPosition(.88);
            intakeInner.setPower(.65);
            intakeOuter.setPower(-.65);
            indicatorLight1.setPosition(GREEN);
            indicatorLight2.setPosition(GREEN);
        } else if (gateOpen && getRuntime() - savedRuntime >= 2) {
            // do NOT flip gateOpen here
            gate.setPosition(.5);
            indicatorLight1.setPosition(BLUE);
            indicatorLight2.setPosition(BLUE);
            intakeInner.setPower(.65);
            gateOpen = false;
        }

        if (intakeOn && !intakeFull && !gateOpen)       intakeOuter.setPower(-.8);
        else if (intakeOn && intakeFull && !gateOpen)   intakeOuter.setPower(0);
        else if (!intakeOn)                             intakeOuter.setPower(0);

        if (flywheelOn) {
            if (currentFlywheelVelocity < flywheelVelocity) {
                currentFlywheelVelocity = Math.min(currentFlywheelVelocity + FLYWHEEL_RAMP_STEP, flywheelVelocity);
            } else if (currentFlywheelVelocity > flywheelVelocity) {
                currentFlywheelVelocity = Math.max(currentFlywheelVelocity - FLYWHEEL_RAMP_STEP, flywheelVelocity);
            }
        } else {
            currentFlywheelVelocity = Math.max(currentFlywheelVelocity - FLYWHEEL_RAMP_STEP, 0);
        }
        flywheelLeft.setVelocity(currentFlywheelVelocity);
        flywheelRight.setVelocity(currentFlywheelVelocity);

    }

    private void driveRobot() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x *1.2;
        // dampening rotation a lot less
        double rx = gamepad1.right_stick_x *.7 ;

        driving = Math.abs(y) > .03 || Math.abs(x) > .03 || Math.abs(rx) > .03;

        // only normalize if sum exceeds 1.0
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        frontLeftMotor.setPower((y + x + rx) / denominator);
        backLeftMotor.setPower((y - x + rx) / denominator);
        frontRightMotor.setPower((y - x - rx) / denominator);
        backRightMotor.setPower((y + x - rx) / denominator);
    }

    /* ================= APRILTAG ================= */

    private void trackAprilTag() {
        // re-enable if we get it done
    }

    /* ================= TELEMETRY ================= */

    private void updateTelemetry() {
        long now = System.currentTimeMillis();
        long loopTime = now - lastLoopTime;
        lastLoopTime = now;

        telemetry.addData("x: ", follower.getPose().getX());
        telemetry.addData("y: ", follower.getPose().getY());
        telemetry.addData("Flywheel target", flywheelVelocity);
        telemetry.addData("Flywheel actual", currentFlywheelVelocity);
        telemetry.update();
    }

    public void stop() {
        // re-enable if we get it done
    }

    public void trackTarget() {
        // Placeholder
    }
}