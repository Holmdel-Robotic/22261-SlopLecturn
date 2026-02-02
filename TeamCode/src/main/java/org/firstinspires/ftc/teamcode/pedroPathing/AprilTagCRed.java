package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "AprilTagCRed")
public class AprilTagCRed extends LinearOpMode {

    private double max = 0.01;
    private double kP = 0.095;
    private double kI = 0.00;
    private double kD = 0.00;
    private double kF = 0.00;

    private double iSum = 0;
    private double lError = 0;

    @Override
    public void runOpMode() {
        Servo raxon = hardwareMap.get(Servo.class, "raxon");
        Servo laxon = hardwareMap.get(Servo.class, "laxon");
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "absEncoder");
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(1);
        limelight.start();

        double pos = .48;
        raxon.setPosition(pos);
        laxon.setPosition(pos);

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
                double error = result.getFiducialResults().get(0).getTargetXDegrees();

                if (Math.abs(error) > 7) {

                    double d = error - lError;
                    iSum += error;

                    double c = (kP * error) + (kI * iSum) + (kD * d) + kF;
                    lError = error;
                    c = Math.max(-max, Math.min(max, c));

                    pos += c;
                    pos = Math.max(0.0, Math.min(1.0, pos));
                    laxon.setPosition(pos);
                    raxon.setPosition(pos);

                    telemetry.addData("Target ID", result.getFiducialResults().get(0).getFiducialId());
                    telemetry.addData("Error", "%.2f°", error);
                    telemetry.addData("P term", "%.4f", kP * error);
                    telemetry.addData("D term", "%.4f", kD * d);
                    telemetry.addData("Correction", "%.4f", c);
                    telemetry.addData("Position", "%.4f", pos);
                    telemetry.addData("Encoder", "%.0f°", (encoder.getVoltage() / 3.3) * 360);
                    telemetry.addData("Status", "Tracking...");
                }

                else {
                    telemetry.addLine("LOCKED ON");
                    telemetry.addData("Error", "%.2f°", error);
                    lError = 0;
                    iSum = 0;
                }

                telemetry.update();
            }

            sleep(10);
        }
    }
}