package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

@Config
@TeleOp
public class ChatgptBlueTeleopLL extends OpMode {

    private double flV, blV, frV, brV;

    private double robotXPos, robotYPos, robotHeading;
    private Follower follower;

    public double testMillis, lastTestMillis = getRuntime();



    private double processgamepadMillis;

    private double updateRobotStateMillis;

    private double writeHardwareMillis;

    private VoltageSensor myControlHubVoltageSensor;

    private double driveRobotMillis;

    private double lastUpdateTime = getRuntime(), lastProcessgamepadMillis = getRuntime(), lastUpdateRobotStateMillis = getRuntime(), lastWriteHardwareMillis = getRuntime(), lastDriveRobotMillis = getRuntime();



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

    //private Limelight3A limelight;

    private boolean gateOpen = false;

    private boolean debounceDistance = false;

    private boolean driving;
    public static boolean intakeOn = false;
    public static boolean flywheelOn = false;
    public static boolean aprilTagTracking = false;

    private boolean debounceA, debounceX, debounceRightStick, debounceBack, debounceLEFT_TRIGGER, debounceY, debounceDpad_up,debounceDpad_down,debounceDpad_left,debbounceDpad_right;

    public static double flywheelVelocity = 800;
    public static double hoodPos;
    public static double raxonPos = .48;
    public static double laxonPos = .48;

    public static double axonPos = .48;

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

        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //limelight.pipelineSwitch(1);
        //limelight.start();
        stop();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(144-84, 36, Math.toRadians(180)));

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
        //lastProcessgamepadMillis = getRuntime();
        processGamepad1();
        //processgamepadMillis = getRuntime() - lastProcessgamepadMillis;
        //lastUpdateRobotStateMillis = getRuntime();
        updateRobotState();
        //updateRobotStateMillis = getRuntime() - lastUpdateRobotStateMillis;
        //lastWriteHardwareMillis = getRuntime();
        writeHardware();
        //writeHardwareMillis = getRuntime() - lastWriteHardwareMillis;
        //lastDriveRobotMillis = getRuntime();
        driveRobot();
        //driveRobotMillis = getRuntime() - lastDriveRobotMillis;

        follower.update();
        updateTelemetry();
    }

    /* ================= INPUT PROCESSING ================= */

    private void processGamepad1() {

        if (gamepad1.start){
            turnToAngle(Math.toDegrees(Math.atan(((144 -follower.getPose().getY())/follower.getPose().getX()))) + 90);
            // fiugre out the right formula
        }

        if (gamepad1.dpad_up && !debounceDpad_up){
            flywheelVelocity = flywheelVelocity + 50;
            debounceDpad_up = true;

        }
        if (gamepad1.dpad_down && !debounceDpad_down){
            flywheelVelocity = flywheelVelocity - 50;
            debounceDpad_down = true;

        }
        if (gamepad1.dpad_left && !autoAim && !debounceDpad_left){
            hoodPos += .02;
            debounceDpad_left = true;
        }
        if (gamepad1.dpad_right && !autoAim && !debbounceDpad_right){
            hoodPos -= .02;
            debbounceDpad_right = true;
        }
        if (!gamepad1.dpad_up){
            debounceDpad_up = false;
        }
        if (!gamepad1.dpad_down){
            debounceDpad_down = false;
        }
        if (!gamepad1.dpad_left){
            debounceDpad_left = false;
        }
        if (!gamepad1.dpad_right){
            debbounceDpad_right = false;
        }
        if (gamepad1.y && !debounceY){
            autoAim = !autoAim;
            flywheelVelocity = 800;
            debounceY = true;
        }
        if (!gamepad1.y){

            debounceY = false;
        }




        if (gamepad1.back && !debounceBack) {
            savedRuntime = getRuntime();
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
            raxon.setPosition(axonPos);
            laxon.setPosition(axonPos);
            //stop();

        }

        if (gamepad1.guide){
            //stop();
            //limelight.stop();
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

        if (!gamepad1.right_stick_button) debounceRightStick = true;

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
            robotXPos = pose.getX();
            robotYPos = pose.getY();
            robotHeading = pose.getHeading();

            double dy = 144 - robotYPos;
            distance = Math.sqrt(dy * dy + robotXPos * robotXPos);

//        hoodPos = -0.002005998 * distance + (1 - 0.337882)  ;
            hoodPos = -0.004005998 * distance + (1);
//        flywheelVelocity = 2.88 * distance + 1531.52943;
            //flywheelVelocity = 7.75 * (distance) + 925;
            lastUpdateTime = getRuntime();
        }

        testMillis = getRuntime() - lastTestMillis;

//        if (hoodPos < .4) hoodPos = .4;

        //if (aprilTagTracking && loopCount % 2 == 0 && !driving) {
        //    trackAprilTag();
        //}





    }

    /* ================= HARDWARE WRITES ================= */

    private void writeHardware() {


        hood.setPosition(hoodPos);

        if (driving && autoAim){
            raxon.setPosition(axonPos);
            laxon.setPosition(axonPos);
        }


        /*
        if (innerSensorDist < 12 && gateOpen && !debounceDistance ) {
            intakeInner.setPower(0);
            debounceDistance = true;
            savedRuntime = getRuntime();
        } else if (debounceDistance && getRuntime() - savedRuntime >= .5 && innerSensorDist < 12) {
            intakeInner.setPower(.3);

        } else if (debounceDistance && getRuntime() - savedRuntime >= .5 && innerSensorDist > 12) {
            debounceDistance = false;
        }

        */

        if (gateOpen && getRuntime() - savedRuntime < 2) {
            flywheelOn = true;
            gate.setPosition(.88);
            intakeInner.setPower(.65);
            intakeOuter.setPower(-.65);
            indicatorLight1.setPosition(GREEN);
            indicatorLight2.setPosition(GREEN);
        }
//        else if (gateOpen && getRuntime() - savedRuntime < 1 && getRuntime() - savedRuntime > .5) intakeOuter.setPower(0);

//        else if (gateOpen && getRuntime() - savedRuntime < 3) intakeOuter.setPower(-.65);

        else{
            gate.setPosition(.5);
            indicatorLight1.setPosition(BLUE);
            indicatorLight2.setPosition(BLUE);
            intakeInner.setPower(.65);
            gateOpen = !gateOpen;
        }

        if ((intakeOn && !intakeFull && !gateOpen)) intakeOuter.setPower(-.8);
        else if (intakeOn && intakeFull && !gateOpen) intakeOuter.setPower(0);
        else if (!intakeOn) intakeOuter.setPower(0);


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

    /* ================= DRIVE ================= */

    private void driveRobot() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x * .8;

        if (y < .03 && x < .03 && rx < .03) driving = false;
        else driving = true;


        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        frontLeftMotor.setVelocity(2200 * (y + x + rx) / denominator);
        flV = (y + x + rx) / denominator;
        backLeftMotor.setVelocity(2200 * (y - x + rx) / denominator);
        blV = (y - x + rx) / denominator;
        frontRightMotor.setVelocity(2200 * (y - x - rx) / denominator);
        frV = (y - x - rx) / denominator;
        backRightMotor.setVelocity(2200 *(y + x - rx) / denominator);
        brV = (y + x - rx) / denominator;
    }

    /* ================= APRILTAG ================= */

    private void trackAprilTag() {
        /*
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

         */
    }

    /* ================= TELEMETRY ================= */

    private void updateTelemetry() {

        long now = System.currentTimeMillis();
        long loopTime = now - lastLoopTime;
        lastLoopTime = now;


        //telemetry.addData("AprilTracking?",aprilTagTracking);
        //telemetry.addData("FlywheelOn", flywheelOn);
        //telemetry.addData("IntakeOn", intakeOn);

       // telemetry.addData("Inner Dist sensor",innerSensor.getDistance(DistanceUnit.CM));
       // telemetry.addData("intake full?", intakeFull);
      // telemetry.addData("Intake1dist",intake1Dist);
      //  telemetry.addData("intake2dist",intake2Dist);


       // telemetry.addData("FL current", frontLeftMotor.getCurrent(CurrentUnit.AMPS));
       // telemetry.addData("FR current", frontRightMotor.getCurrent(CurrentUnit.AMPS));
       // telemetry.addData("BL current", backLeftMotor.getCurrent(CurrentUnit.AMPS));
       // telemetry.addData("BR current", backRightMotor.getCurrent(CurrentUnit.AMPS));

       // telemetry.addData("Flywheel L Current", flywheelLeft.getCurrent(CurrentUnit.AMPS));
        //telemetry.addData("Flywheel R Current", flywheelRight.getCurrent(CurrentUnit.AMPS));
       // telemetry.addData("innerIntake", intakeInner.getCurrent(CurrentUnit.AMPS));
       // telemetry.addData("OuterIntake", intakeOuter.getCurrent(CurrentUnit.AMPS));

        //telemetry.addData("DriveMillis", driveRobotMillis *1000);
        //telemetry.addData("ProcessGamepadMillis", processgamepadMillis *1000);
        //telemetry.addData("writeHardwareMillis", writeHardwareMillis *1000);
        //telemetry.addData("updateRobotStateMillis",updateRobotStateMillis *1000);
        //telemetry.addData("testMillis", testMillis);


      //  telemetry.addData("Loop ms", loopTime);
      //  telemetry.addData("Hz", 1000.0 / loopTime);
      //  telemetry.addData("FlywheelV L", (flywheelLeft.getVelocity()));
      //  telemetry.addData("FlywheelV R", (flywheelRight.getVelocity()));
       // telemetry.addData("flyWheel V intended", flywheelVelocity);
      //  telemetry.addData("Distance",distance);
       // telemetry.addData("Hood Pos", hood.getPosition());
       // telemetry.addData("AutoAim", autoAim);
       // telemetry.addData("dUp", debounceDpad_up);
      //  telemetry.addData("dDown", debounceDpad_down);
        telemetry.addData("x: ", follower.getPose().getX());
        telemetry.addData("y: ", follower.getPose().getY());
       // telemetry.addData("theta: ", Math.toDegrees(follower.getPose().getHeading()));
       // telemetry.addData("autoFly", getRuntime() - lastUpdateTime > .05 && autoAim);

        telemetry.addData("theta: ", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("autoFly", getRuntime() - lastUpdateTime > .05 && autoAim);

        telemetry.update();
    }

    public void stop(){
        /*
        try {
            if (limelight != null) limelight.stop();
        } catch (Exception ignored) {}

         */
    }

    public void turnToAngle(double TargetToDegrees){
        if (TargetToDegrees - Math.toDegrees(robotHeading) > 0){
            while (Math.abs(Math.toDegrees(follower.getPose().getHeading()) - TargetToDegrees) > 10){
                frontLeftMotor.setPower(.3);
                backLeftMotor.setPower(.3);
                frontRightMotor.setPower(-.3);
                backRightMotor.setPower(-.3);
                follower.update();
            }

        } else {
            while (Math.abs(Math.toDegrees(follower.getPose().getHeading()) - TargetToDegrees) > 10) {
                frontLeftMotor.setPower(-.3);
                backLeftMotor.setPower(-.3);
                frontRightMotor.setPower(.3);
                backRightMotor.setPower(.3);
                follower.update();
            }

        }

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public void trackTarget(){

    }


}
