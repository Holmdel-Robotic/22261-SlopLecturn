package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.PIDController;

//use dash for ts
//http://192.168.43.1:8080/dash
//live tuning ty
@Config
@TeleOp(name = "TurretPID", group = "Test")
public class TurretPidTester extends LinearOpMode {

    public static boolean enabled = true;

    public static double minDeg = 0.0;     // min without wire getting cooked pls check angle
    public static double maxDeg = 300.0;   // max without wire getting cooked pls check angle
    public static double targetDeg = 150.0;

    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0;

    private double estDeg = 0; // assumes turret starts facing forward pls reset manually before testing
    private Servo raxon, laxon;
    private PIDController pid;

    @Override
    public void runOpMode() {
        raxon = hardwareMap.get(Servo.class, "raxon");
        laxon = hardwareMap.get(Servo.class, "laxon");

        pid = new PIDController(kP, kI, kD);

        estDeg = clamp(0, minDeg, maxDeg);
        setTurretDeg(estDeg);

        waitForStart();

        while (opModeIsActive()) {

            pid.setPIDConstants(kP, kI, kD);

            double tgt = clamp(targetDeg, minDeg, maxDeg);

            if (enabled) {
                double error = tgt - estDeg;

                double deltaDeg = pid.calculate(error);

                estDeg = clamp(estDeg + deltaDeg, minDeg, maxDeg);
                setTurretDeg(estDeg);
            }

            telemetry.addData("enabled", enabled);
            telemetry.addData("targetDeg", "%.2f (clamped %.2f)", targetDeg, tgt);
            telemetry.addData("estDeg", "%.2f", estDeg);
            telemetry.addData("error", "%.2f", tgt - estDeg);
            telemetry.addData("raxon", "%.3f", raxon.getPosition());
            telemetry.addData("laxon", "%.3f", laxon.getPosition());
            telemetry.update();
        }
    }

    private void setTurretDeg(double deg) {
        deg = clamp(deg, minDeg, maxDeg);

        double t = (deg - minDeg) / (maxDeg - minDeg);
        t = clamp(t, 0.0, 1.0);

        raxon.setPosition(t);
        laxon.setPosition(t);
    }

    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }
}