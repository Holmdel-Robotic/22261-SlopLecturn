package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
@TeleOp
public class ChatgptBlueTeleopLL extends OpMode {

    private Follower follower;


    private Pose pose;

    private Servo gate, indicatorLight1, indicatorLight2;
    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    private DcMotorEx flywheelLeft, flywheelRight, intakeOuter, intakeInner;

    private DistanceSensor intakeSensor1, intakeSensor2;

    private Servo hood, raxon, laxon;

    private Limelight3A limelight;

    private boolean gateOpen = false;

    private boolean intakeOn = false;
    private boolean flywheelOn = false;
    private boolean aprilTagTracking = false;

    private boolean debounceA, debounceX, debounceRightStick, debounceBack, debounceLEFT_TRIGGER;

    private double flywheelVelocity = 1600;
    private double hoodPos;
    private double raxonPos = .48;
    private double laxonPos = .48;

    private int loopCount = 0;
    private long lastLoopTime;

    private double kP = 0.843;
    private double max = 0.00922;

    private double GREEN = .5;
    private double BLUE = .6;

    private double distance;
    @Override
    public void init() {



        gate = hardwareMap.get(Servo.class, "gate");

        indicatorLight1 = hardwareMap.get(Servo.class, "lightOne");
        indicatorLight2 = hardwareMap.get(Servo.class, "lightTwo");

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fl");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fr");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "bl");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "br");

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

        hood = hardwareMap.get(Servo.class, "hood");
        raxon = hardwareMap.get(Servo.class, "raxon");
        laxon = hardwareMap.get(Servo.class, "laxon");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(144 - 84, 36, Math.toRadians(180)));

        lastLoopTime = System.currentTimeMillis();

        indicatorLight1.setPosition(BLUE);
        indicatorLight2.setPosition(BLUE);

        raxon.setPosition(.48);
        laxon.setPosition(.48);
        raxonPos = .48;
        laxonPos = .48;

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



        if (gamepad1.back && !debounceBack) {
            gateOpen = !gateOpen;
            debounceBack = true;
        }

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

        if (!gamepad1.back) debounceBack = false;

        if (gamepad1.right_stick_button && debounceRightStick) {
            aprilTagTracking = !aprilTagTracking;
            debounceRightStick = false;
        }
        if (!aprilTagTracking) {
            if (gamepad1.left_trigger > .01 && debounceLEFT_TRIGGER) {
                raxonPos = raxon.getPosition() + .03;
                laxonPos = laxon.getPosition() - .03;
                laxon.setPosition(laxonPos);
                raxon.setPosition(raxonPos);

                debounceLEFT_TRIGGER = false;
            }
        }
        if (!gamepad1.right_stick_button) debounceRightStick = true;
    }

    /* ================= ROBOT LOGIC ================= */

    private void updateRobotState() {

        pose = follower.getPose();
        double x = pose.getX();
        double y = pose.getY();

        double dy = 144 - y;
        distance = Math.sqrt(dy * dy + x * x);

        hoodPos = 0.000705998 * distance + 0.337882;
        flywheelVelocity = 2.95 * distance + 1531.52943;

        if (hoodPos < .17) hoodPos = .17;

        if (aprilTagTracking) {
            trackAprilTag();
        }
    }

    /* ================= HARDWARE WRITES ================= */

    private void writeHardware() {

        double intake1Dist = intakeSensor1.getDistance(DistanceUnit.CM);
        double intake2Dist = intakeSensor2.getDistance(DistanceUnit.CM);
        boolean intakeFull = intake1Dist < 15 && intake2Dist < 15;

        hood.setPosition(hoodPos);

        if (gateOpen){
            gate.setPosition(.88);
            indicatorLight1.setPosition(GREEN);
            indicatorLight2.setPosition(GREEN);
        }
        else{
            gate.setPosition(.5);
            indicatorLight1.setPosition(BLUE);
            indicatorLight2.setPosition(BLUE);
        }
        if ((intakeOn && !intakeFull) || gateOpen) intakeOuter.setPower(-.8);
        else intakeOuter.setPower(0);

        if (gateOpen) intakeInner.setPower(.45);
        else intakeInner.setPower(0);

        if (flywheelOn) {
            flywheelLeft.setVelocity(flywheelVelocity);
            flywheelRight.setVelocity(flywheelVelocity);
        } else {
            flywheelLeft.setVelocity(0);
            flywheelRight.setVelocity(0);
        }
    }

    /* ================= DRIVE ================= */

    private void driveRobot() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x * .4;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        frontLeftMotor.setPower((y + x + rx) / denominator);
        backLeftMotor.setPower((y - x + rx) / denominator);
        frontRightMotor.setPower((y - x - rx) / denominator);
        backRightMotor.setPower((y + x - rx) / denominator);
    }

    /* ================= APRILTAG ================= */

    private void trackAprilTag() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            double error = result.getFiducialResults().get(0).getTargetXDegrees();

            if (Math.abs(error) > 3) {
                double correction = kP * error;
                correction = Math.max(-max, Math.min(max, correction));

                raxonPos += correction;
                laxonPos += correction;
                raxonPos = Math.max(0.1, Math.min(1.0, raxonPos));
                laxonPos = Math.max(0.1, Math.min(1.0, laxonPos));

                laxon.setPosition(laxonPos);
                raxon.setPosition(raxonPos);

                telemetry.addData("AT Target ID", result.getFiducialResults().get(0).getFiducialId());
                telemetry.addData("AT Error", "%.2f°", error);
                telemetry.addData("AT Correction", "%.5f", correction);
                telemetry.addData("AT Status", "TRACKING");
            } else {
                telemetry.addData("AT Status", "LOCKED ON");
                telemetry.addData("AT Error", "%.2f°", error);
            }
        } else {
            telemetry.addData("AT Status", "Searching...");
        }
    }

    /* ================= TELEMETRY ================= */

    private void updateTelemetry() {

        long now = System.currentTimeMillis();
        long loopTime = now - lastLoopTime;
        lastLoopTime = now;
        telemetry.addData("AprilTracking?",aprilTagTracking);
        telemetry.addData("Loop ms", loopTime);
        telemetry.addData("Hz", 1000.0 / loopTime);
        telemetry.addData("FlywheelOn", flywheelOn);
        telemetry.addData("IntakeOn", intakeOn);
        telemetry.addData("FlywheelV", (flywheelLeft.getVelocity() + flywheelRight.getVelocity())/2);
        telemetry.addData("Distance",distance);
        telemetry.addData("Hood Pos", hood.getPosition());
        telemetry.update();
    }

    public void stop(){
        try {
            if (limelight != null) limelight.stop();
        } catch (Exception ignored) {}
    }
}
