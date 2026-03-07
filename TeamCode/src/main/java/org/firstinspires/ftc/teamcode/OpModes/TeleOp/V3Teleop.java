package org.firstinspires.ftc.teamcode.OpModes.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

@Config
@TeleOp


public class V3Teleop extends OpMode {
    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor, intakeOuter,intakeInner, flywheelLeft, flywheelRight;

    private Servo hood, raxon, laxon;

    public static double IntendedFlywheelV = 1600, ServoPos = .5;

    private boolean debounceDPAD, debounceX, FlywheelOn;
    public static double testPos = 0.5;

    public void init(){
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

        FlywheelOn = false;


    }

    public void loop(){
     HandleInputs();
     driveRobot();
     OuterIntakeOperation(gamepad1.b);
     WholeIntakeOperation(gamepad1.a);
     changeFlywheelVelo();
     SetFlywheelVelocity(IntendedFlywheelV);
     telemetry();
     smoothServo(ServoPos);





    }



    private void driveRobot() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x * .8;


        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        frontLeftMotor.setPower((y + x + rx) / denominator);

        backLeftMotor.setPower((y - x + rx) / denominator);

        frontRightMotor.setPower((y - x - rx) / denominator);

        backRightMotor.setPower((y + x - rx) / denominator);

    }

    private void WholeIntakeOperation(boolean gamepad){
        if (gamepad) {
            intakeInner.setPower(.75);
            intakeOuter.setPower(.75);
        }
        else{
            intakeInner.setPower(0);
        }
    }

    private void OuterIntakeOperation(boolean gamepad){
        if (gamepad) {
            intakeOuter.setPower(.75);
        }
        else{
            intakeOuter.setPower(0);
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
        if (gamepad1.x && !debounceX){
            FlywheelOn = !FlywheelOn;
            debounceX = true;
        }

        if (!gamepad1.x){
            debounceX = false;
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

    private void smoothServo(double pos){
        double lastloopTime = getRuntime();
        while (Math.abs(pos - laxon.getPosition()) > .04){
            if (getRuntime() - lastloopTime >= .2){
                if (pos > laxon.getPosition()){
                    setServoPos(laxon.getPosition() + .04);

                } else if (pos < laxon.getPosition()) {
                    setServoPos(laxon.getPosition() - .04);
                }
                lastloopTime = getRuntime();
            }
        }
    }

    private void telemetry(){
        telemetry.addData("laxon pos", laxon.getPosition());
        telemetry.addData("raxon pos", raxon.getPosition());
        telemetry.addData("runtime", getRuntime());

    }

}

