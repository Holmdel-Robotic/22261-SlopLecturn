package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class AxonTest extends LinearOpMode {
    public static double testPos = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {
        Servo s1 = hardwareMap.get(Servo.class, "raxon");
        Servo s2 = hardwareMap.get(Servo.class,"laxon");
        waitForStart();

        while(opModeIsActive()){
            s1.setPosition(testPos);
            s2.setPosition(testPos);
        }
    }






}


