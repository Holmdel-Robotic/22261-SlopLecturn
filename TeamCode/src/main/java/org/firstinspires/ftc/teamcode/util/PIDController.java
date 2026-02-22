package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PIDController {

    public double kP;
    public double kI;
    public double kD;

    private double integral = 0.0;
    private double lastError = 0.0;

    public PIDController(double kP, double kI, double kD) {
        setPIDConstants(kP, kI, kD);
    }

    public double calculate(double error) {
        integral += error;
        double derivative = error - lastError;
        lastError = error;
        return (kP * error) + (kI * integral) + (kD * derivative);
    }

    public void reset() {
        integral = 0.0;
        lastError = 0.0;
    }

    public void setPIDConstants(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
    public static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }
}