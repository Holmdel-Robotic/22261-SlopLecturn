package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//try?


@Config
@TeleOp


public class V3TeleopRed extends OpMode {


    private Follower follower;
    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor, intakeOuter,intakeInner, flywheelLeft, flywheelRight;

    private Servo hood, raxon, laxon, innerGate, outerGate;

    public static double IntendedFlywheelV = 1600, ServoPos = .5, desiredAngle, Heading, hoodPos = .53, XOffset = 0, distanceToGoal, DegreeOffset = -20, savedTime = 0, p, i, d, previous_error, previous_time, k_p, k_i, k_d, max_i;

    public static boolean autoTarget = true;

    private boolean debounceDPAD, debounceX, FlywheelOn, debounceB, outerIntakeOn, DriveMode = true, debounceY, dLTR, dRTR, dRBR, dLBR;
    public static double testPos = 0.5;
    private double open, close;




    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(84, 36, Math.toRadians(0)));
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
        hood.setDirection(Servo.Direction.REVERSE);

        FlywheelOn = false;


    }

    public void loop(){

        follower.update();
        OuterIntakeOperation(gamepad1.b);
        WholeIntakeOperation(gamepad1.right_trigger > .2);
        HandleInputs();
        driveRobot();

        changeFlywheelVelo();
        SetFlywheelVelocity(IntendedFlywheelV);
        telemetry();
        calculateCorrectAngle();
        setServoPos(ServoPos);
        hood.setPosition(hoodPos);
        telemetry.addData("looptime", getRuntime() - savedTime);
        telemetry.addData("offset", DegreeOffset);
        savedTime = getRuntime();




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
            intakeInner.setPower(.9);
            intakeOuter.setPower(.9);
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
            outerIntakeOn = !outerIntakeOn;
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
            IntendedFlywheelV = 1750;
            hoodPos = .53;
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
        if (gamepad1.left_trigger > .2){
            outerGate.setPosition(.575);
            innerGate.setPosition(.2);
            intakeInner.setPower(.9);
            intakeOuter.setPower(.9);
        }


        if(gamepad1.left_trigger == 0)
        {
            dLTR = false;
        }
        //bait

        if(gamepad1.right_trigger == 0)
        {
            dRTR = false;
        }




    }

    private void changeFlywheelVelo() {
        if (FlywheelOn) {
            flywheelLeft.setVelocity(IntendedFlywheelV);
            flywheelRight.setVelocity(IntendedFlywheelV);
        }
        else {
            flywheelLeft.setVelocity(0);
            flywheelRight.setVelocity(0);
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
    telemetry.addData("flywheelLeft", flywheelLeft.getVelocity());
    telemetry.addData("flywheelRight", flywheelRight.getVelocity());
    telemetry.addData("distance to goal", distanceToGoal);
    telemetry.addData("fl current", frontLeftMotor.getCurrent(CurrentUnit.AMPS));
    telemetry.addData("bl current", backLeftMotor.getCurrent(CurrentUnit.AMPS));
    telemetry.addData("fr current", frontRightMotor.getCurrent(CurrentUnit.AMPS));
    telemetry.addData("br current", backRightMotor.getCurrent(CurrentUnit.AMPS));
    telemetry.addData("intake Outer", intakeOuter.getCurrent(CurrentUnit.AMPS));
    telemetry.addData("intake inner", intakeInner.getCurrent(CurrentUnit.AMPS));
    telemetry.addData("flywheel L Current", flywheelLeft.getCurrent(CurrentUnit.AMPS));
    telemetry.addData("flywheel R current", flywheelRight.getCurrent(CurrentUnit.AMPS));


}

private void calculateCorrectAngle(){

    follower.update();
    Pose currPose = follower.getPose();
    distanceToGoal = Math.sqrt(Math.pow(144-currPose.getY(),2) + Math.pow(currPose.getX(),2));


    if (autoTarget) {
        //hoodPos = .00263636 * distanceToGoal - .02182;
        if ((currPose.getY() < 30 && currPose.getX() > 42 && currPose.getX() < 103) || (currPose.getY() > 70)){

            telemetry.addData("X dist", currPose.getX());
            telemetry.addData("Y dist", currPose.getY());
            Heading = Math.toDegrees(currPose.getHeading());

            if (Heading < 0) {
                Heading = 360 + Heading;
            }


            desiredAngle = 180 + (Math.toDegrees(Math.atan((144 - currPose.getY()) / (144 - currPose.getX() - XOffset)))) - Heading;
            desiredAngle = (int) desiredAngle + DegreeOffset;

            if (desiredAngle < 0) {
                desiredAngle = 360 + desiredAngle;
            }
            ServoPos = 0.00338889 * desiredAngle - 0.0366667;

            distanceToGoal = Math.sqrt(Math.pow(144 - currPose.getY(), 2) + Math.pow(144 - currPose.getX(), 2));
            IntendedFlywheelV = .037793 * Math.pow(distanceToGoal, 2) - 0.153573 * distanceToGoal + 1199.55171;

            if (gamepad1.dpad_left && !debounceDPAD){
                DegreeOffset += 5;
                debounceDPAD = true;
            }

            if (gamepad1.dpad_right && ! debounceDPAD){
                DegreeOffset -= 5;
                debounceDPAD = true;

            }

            if (!gamepad1.dpad_right && !gamepad1.dpad_left && !gamepad1.dpad_up && !gamepad1.dpad_down){
                debounceDPAD = false;
            }


        }
    }
    else{


            /*
            if(gamepad1.left_trigger > 0 && !dLTR){
                ServoPos -= .02;
                dLTR = true;
            }
            */
        if(gamepad1.left_trigger == 0)
        {
            dLTR = false;
        }
            /*
            if(gamepad1.right_trigger > 0 && !dRTR){
                ServoPos += .02;
                dRTR = true;
            }

             */

        if(gamepad1.left_bumper  && !dLBR)
        {
            hoodPos -= .05;
        }
        if(!gamepad1.left_bumper)
        {

            dLBR = false;
        }
            /*
            if(gamepad1.right_trigger == 0)
            {
                dRTR = false;
            }

            if(gamepad1.right_bumper  && !dRBR)
            {

                hoodPos += .05;
            }
            */

        if(!gamepad1.right_bumper)
        {
            dRBR = false;
        }
    }

}

}


