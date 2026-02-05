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

    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    private DcMotorEx flywheelLeft, flywheelRight, intakeOuter, intakeInner;

    private DistanceSensor intakeSensor1, intakeSensor2;

    private Servo hood, raxon, laxon;

    private Limelight3A limelight;

    private boolean intakeOn = false;
    private boolean flywheelOn = false;
    private boolean aprilTagTracking = false;

    private boolean debounceA, debounceX, debounceRightStick;

    private double flywheelVelocity = 1600;
    private double hoodPos;
    private double raxonPos = .48;
    private double laxonPos = .48;

    private int loopCount = 0;
    private long lastLoopTime;

    private double kP = 0.08;
    private double max = 0.00962;

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

        hood = hardwareMap.get(Servo.class, "hood");
        raxon = hardwareMap.get(Servo.class, "raxon");
        laxon = hardwareMap.get(Servo.class, "laxon");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        follower = Constants.createFollower(hardwareMap);

        lastLoopTime = System.currentTimeMillis();
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
    }

    /* ================= ROBOT LOGIC ================= */

    private void updateRobotState() {

        Pose pose = follower.getPose();
        double x = pose.getX();
        double y = pose.getY();

        double dy = 144 - y;
        double distance = Math.sqrt(dy * dy + x * x);

        hoodPos = 0.000705998 * distance + 0.337882;
        flywheelVelocity = 3 * distance + 1531.52943;

        if (hoodPos < .17) hoodPos = .17;

        if (aprilTagTracking && loopCount % 2 == 0) {
            trackAprilTag();
        }
    }

    /* ================= HARDWARE WRITES ================= */

    private void writeHardware() {

        double intake1Dist = intakeSensor1.getDistance(DistanceUnit.CM);
        double intake2Dist = intakeSensor2.getDistance(DistanceUnit.CM);
        boolean intakeFull = intake1Dist < 15 && intake2Dist < 15;

        hood.setPosition(hoodPos);

        if (intakeOn && !intakeFull) intakeOuter.setPower(-.8);
        else intakeOuter.setPower(0);

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
            double correction = Math.max(-max, Math.min(max, kP * error));

            raxonPos += correction;
            laxonPos += correction;

            raxon.setPosition(raxonPos);
            laxon.setPosition(laxonPos);
        }
    }

    /* ================= TELEMETRY ================= */

    private void updateTelemetry() {

        long now = System.currentTimeMillis();
        long loopTime = now - lastLoopTime;
        lastLoopTime = now;

        telemetry.addData("Loop ms", loopTime);
        telemetry.addData("Hz", 1000.0 / loopTime);
        telemetry.addData("FlywheelOn", flywheelOn);
        telemetry.addData("IntakeOn", intakeOn);
        telemetry.update();
    }
}
