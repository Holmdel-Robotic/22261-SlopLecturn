package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
@TeleOp
public class ChatgptBlueTeleopLL extends OpMode {

    private long lastLoopTime;
    private int loopCount = 0;

    private Follower follower;
    private TelemetryManager telemetryM;

    private DistanceSensor intakeSensor1, intakeSensor2, distanceSensor;
    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    private DcMotorEx flywheelLeft, flywheelRight, intakeOuter, intakeInner;

    private Servo gate, hood, blocker, raxon, laxon, indicatorLight1, indicatorLight2;
    private IMU imu;
    private Limelight3A limelight;

    private boolean intakeOn, flywheelOn, feederOn, aprilTagTracking;
    private boolean debounceA, debounceB, debounceX, debounceRightStick;

    private double flywheelVelocity = 1600;
    private double hoodPos;
    private double raxonPos = .48;
    private double laxonPos = .48;

    private double kP = 0.08;
    private double max = 0.00962;

    private double cachedHood = -1;
    private double cachedRaxon = -1;
    private double cachedLaxon = -1;

    @Override
    public void init() {

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fl");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fr");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "bl");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "br");

        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flyL");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flyR");

        intakeOuter = hardwareMap.get(DcMotorEx.class, "intOuter");
        intakeInner = hardwareMap.get(DcMotorEx.class, "intInner");

        intakeSensor1 = hardwareMap.get(DistanceSensor.class, "intakeSensor1");
        intakeSensor2 = hardwareMap.get(DistanceSensor.class, "intakeSensor2");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        hood = hardwareMap.get(Servo.class, "hood");
        gate = hardwareMap.get(Servo.class, "gate");
        blocker = hardwareMap.get(Servo.class, "blocker");
        raxon = hardwareMap.get(Servo.class, "raxon");
        laxon = hardwareMap.get(Servo.class, "laxon");

        indicatorLight1 = hardwareMap.get(Servo.class, "lightOne");
        indicatorLight2 = hardwareMap.get(Servo.class, "lightTwo");

        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        limelight.pipelineSwitch(1);
        limelight.start();

        lastLoopTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {

        loopCount++;

        /* ---------------- CACHE READS ---------------- */

        Pose pose = follower.getPose();
        double poseX = pose.getX();
        double poseY = pose.getY();

        double intake1Dist = intakeSensor1.getDistance(DistanceUnit.CM);
        double intake2Dist = intakeSensor2.getDistance(DistanceUnit.CM);
        boolean intakeFull = intake1Dist < 15 && intake2Dist < 15;

        /* ---------------- AUTO TARGET ---------------- */

        double dy = 144 - poseY;
        double distance = Math.sqrt(dy * dy + poseX * poseX);

        hoodPos = 0.000705998 * distance + 0.337882;
        flywheelVelocity = 3 * distance + 1531.52943;

        if (hoodPos < .17) hoodPos = .17;

        if (Math.abs(cachedHood - hoodPos) > 0.002) {
            hood.setPosition(hoodPos);
            cachedHood = hoodPos;
        }

        /* ---------------- INTAKE ---------------- */

        if (intakeOn) {
            if (!intakeFull) {
                intakeOuter.setPower(-.8);
            } else {
                intakeOuter.setPower(0);
            }
        } else {
            intakeOuter.setPower(0);
            intakeInner.setPower(0);
        }

        /* ---------------- FLYWHEEL ---------------- */

        if (flywheelOn) {
            flywheelLeft.setVelocity(flywheelVelocity);
            flywheelRight.setVelocity(flywheelVelocity);
        }

        /* ---------------- APRILTAG ---------------- */

        if (aprilTagTracking && loopCount % 2 == 0) {
            trackAprilTag();
        }

        /* ---------------- DRIVE ---------------- */

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x * .4;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        frontLeftMotor.setPower((y + x + rx) / denominator);
        backLeftMotor.setPower((y - x + rx) / denominator);
        frontRightMotor.setPower((y - x - rx) / denominator);
        backRightMotor.setPower((y + x - rx) / denominator);

        follower.update();

        /* ---------------- TELEMETRY ---------------- */

        long now = System.currentTimeMillis();
        long loopTime = now - lastLoopTime;
        lastLoopTime = now;

        if (loopCount % 2 == 0) {
            telemetry.addData("Loop ms", loopTime);
            telemetry.addData("Hz", 1000.0 / loopTime);
            telemetry.addData("Pose", pose);
            telemetry.addData("IntakeFull", intakeFull);
            telemetry.addData("FlywheelVel", flywheelVelocity);
        }

        telemetry.update();
    }

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

                if (Math.abs(cachedRaxon - raxonPos) > 0.002) {
                    raxon.setPosition(raxonPos);
                    cachedRaxon = raxonPos;
                }

                if (Math.abs(cachedLaxon - laxonPos) > 0.002) {
                    laxon.setPosition(laxonPos);
                    cachedLaxon = laxonPos;
                }
            }
        }
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }
}
