package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class GateTest extends LinearOpMode {
    public static double innerPos = 0, outerPos = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        Servo s1 = hardwareMap.get(Servo.class, "innerGate");
        Servo s2 = hardwareMap.get(Servo.class, "outerGate");

        waitForStart();

        while(opModeIsActive()){
            s1.setPosition(innerPos);
            s2.setPosition(outerPos);

        }
    }






}


