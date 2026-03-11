package org.firstinspires.ftc.teamcode.OpModes.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
@TeleOp


public class V3Teleop extends OpMode {


    private Follower follower;
    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor, intakeOuter,intakeInner, flywheelLeft, flywheelRight;

    private Servo hood, raxon, laxon, innerGate, outerGate;

    public static double IntendedFlywheelV = 1600, ServoPos = .5, desiredAngle, Heading, hoodPos = .5, XOffset = 16;

    public static boolean autoTarget = true;

    private boolean debounceDPAD, debounceX, FlywheelOn, debounceB, outerIntakeOn, DriveMode = true, debounceY, dLTR, dRTR, dRBR, dLBR;
    public static double testPos = 0.5;
    private double open, close;


    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(144-84, 36, Math.toRadians(180)));
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
        intakeInner.setDirection(DcMotor.Direction.REVERSE);
        intakeOuter.setDirection(DcMotor.Direction.FORWARD);
        intakeInner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flyL");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flyR");
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelLeft.setDirection(DcMotor.Direction.FORWARD);
        flywheelRight.setDirection(DcMotor.Direction.REVERSE);

        hood = hardwareMap.get(Servo.class, "hood");
        raxon = hardwareMap.get(Servo.class, "raxon");
        laxon = hardwareMap.get(Servo.class, "laxon");
        innerGate = hardwareMap.get(Servo.class, "innerGate");
        outerGate = hardwareMap.get(Servo.class, "outerGate");

        FlywheelOn = false;


    }

    public void loop(){

     follower.update();
     HandleInputs();
     driveRobot();
     OuterIntakeOperation(gamepad1.b);
     WholeIntakeOperation(gamepad1.a);
     changeFlywheelVelo();
     SetFlywheelVelocity(IntendedFlywheelV);
     telemetry();
     calculateCorrectAngle();
     setServoPos(ServoPos);
     hood.setPosition(hoodPos);




    }



    private void driveRobot() {

        if(DriveMode) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x * .8;


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            frontLeftMotor.setPower((y + x + rx) / denominator);

            backLeftMotor.setPower((y - x + rx) / denominator);

            frontRightMotor.setPower((y - x - rx) / denominator);

            backRightMotor.setPower((y + x - rx) / denominator);
        }else {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.


            double botHeading = follower.getPose().getHeading();

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
        }


    private void WholeIntakeOperation(boolean gamepad){
        if (gamepad) {
            intakeInner.setPower(.75);
            intakeOuter.setPower(.75);
            innerGate.setPosition(.575);
            outerGate.setPosition(.6);
        }
        else{
            intakeInner.setPower(0);
            innerGate.setPosition(.2);
            outerGate.setPosition(0);
        }
    }

    private void OuterIntakeOperation(boolean gamepad){
        if (!gamepad){
            debounceB = true;
        }
        if (gamepad && debounceB) {
            if (outerIntakeOn){
                intakeOuter.setPower(.75);
            }else {
                intakeOuter.setPower(0);
            }
            debounceB = false;
        }
    }

    private void SetFlywheelVelocity(double v){
        if (FlywheelOn) {
            flywheelLeft.setVelocity(v);
            flywheelRight.setVelocity(v);
        }
        else {
            flywheelLeft.setVelocity(0);
            flywheelRight.setVelocity(0);
        }
    }

    private void HandleInputs(){

        if (gamepad1.y && !debounceY){
            autoTarget = !autoTarget;
            debounceY = true;
        }

        if (gamepad1.x && !debounceX){
            FlywheelOn = !FlywheelOn;
            debounceX = true;
        }

        if (!gamepad1.x){
            debounceX = false;
        }

        if (!gamepad1.y){

            debounceY = false;
        }




    }

    private void changeFlywheelVelo(){



        if (gamepad1.dpad_up && !debounceDPAD){
            IntendedFlywheelV += 50;
            debounceDPAD = true;

        } else if (gamepad1.dpad_down && !debounceDPAD) {
            IntendedFlywheelV -= 50;
            debounceDPAD = true;
        }
        if (gamepad1.dpad_down && gamepad1.dpad_up){
            debounceDPAD = false;
        }

    }

    private void setServoPos(double pos){
      laxon.setPosition(pos);
      raxon.setPosition(pos);

    }



    private void telemetry(){
        telemetry.addData("laxon pos", laxon.getPosition());
        telemetry.addData("raxon pos", raxon.getPosition());
        telemetry.addData("flywheel V", (flywheelLeft.getVelocity() + flywheelRight.getVelocity())/2);
        telemetry.addData("runtime", getRuntime());
        telemetry.addData("desired Pos", ServoPos);
        telemetry.addData("desired angle", desiredAngle);
        telemetry.addData("Robot heading", Heading);
        telemetry.addData("alt desired pos", desiredAngle/340);
        telemetry.addData("atan", Math.atan(144 - follower.getPose().getY()/ follower.getPose().getX()));
        telemetry.addData("hood pos", hood.getPosition());
        telemetry.addData("distance from goal", Math.sqrt(Math.pow((144 - follower.getPose().getY()), 2) + Math.pow((follower.getPose().getX()), 2)));
        telemetry.addData("flywheelLeft", flywheelLeft.getVelocity());
        telemetry.addData("flywheelRight", flywheelRight.getVelocity());
    }

    private void calculateCorrectAngle(){
        if (autoTarget) {
            follower.update();
            Pose currPose = follower.getPose();
            desiredAngle = Math.atan(144 - currPose.getY() / currPose.getX());
            telemetry.addData("X dist", currPose.getX());
            telemetry.addData("Y dist", currPose.getY());
            Heading = Math.toDegrees(currPose.getHeading());

            if (Heading < 0) {
                Heading = 360 + Heading;
            }


            desiredAngle = (180 - Math.toDegrees(Math.atan((144 - currPose.getY()) / (currPose.getX() - XOffset)))) - Heading;
            desiredAngle = 180 + (int) desiredAngle;
            ServoPos = 0.00338889 * desiredAngle - 0.0366667;

        }
        else{
            if(gamepad1.left_trigger > 0 && !dLTR){
                ServoPos -= .02;
                dLTR = true;
            }
            if(gamepad1.left_trigger == 0)
            {
                dLTR = false;
            }
            if(gamepad1.right_trigger > 0 && !dRTR){
                ServoPos += .02;
                dRTR = true;
            }
            if(gamepad1.right_trigger == 0)
            {
                dRTR = false;
            }
            if(gamepad1.left_bumper  && !dLBR)
            {
                hoodPos -= .05;
            }
            if(!gamepad1.left_bumper)
            {
                dLBR = false;
            }
            if(gamepad1.right_bumper  && !dRBR)
            {
                hoodPos += .05;
            }
            if(!gamepad1.right_bumper)
            {
                dRBR = false;
            }
        }

    }

}


