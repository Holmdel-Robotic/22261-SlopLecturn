/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "AprilTagCBlue")
public class AprilTagCBlue extends LinearOpMode {

    private double max = 0.01;
    private double kP = 0.07;
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

                if (Math.abs(error) > 7 && Math.abs(lError) > 7 ) {

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
//
//                else if (Math.abs(error) < 7 && Math.abs(error) > 9) {
//
//                    double d = error - lError;
//                    iSum += error;
//
//                    double kp = 0.1;
//
//                    double kd = 0.00;
//
//                    double c = (kp * error) + (kI * iSum) + (kd * d) + kF;
//                    lError = error;
//                    c = Math.max(-max, Math.min(max, c));
//
//                    pos += c;
//                    pos = Math.max(0.0, Math.min(1.0, pos));
//                    laxon.setPosition(pos);
//                    raxon.setPosition(pos);
//
//                    telemetry.addData("Target ID", result.getFiducialResults().get(0).getFiducialId());
//                    telemetry.addData("Error", "%.2f°", error);
//                    telemetry.addData("P term", "%.4f", kP * error);
//                    telemetry.addData("D term", "%.4f", kd * d);
//                    telemetry.addData("Correction", "%.4f", c);
//                    telemetry.addData("Position", "%.4f", pos);
//                    telemetry.addData("Encoder", "%.0f°", (encoder.getVoltage() / 3.3) * 360);
//                    telemetry.addData("Status", "Tracking...");
//                }

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
