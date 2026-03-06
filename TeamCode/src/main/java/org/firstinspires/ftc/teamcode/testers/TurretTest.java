package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="TurretTest")
public class TurretTest extends OpMode {


    public static String SERVO_NAME = "turret";
    public static double turretMinDeg = -180;
    public static double turretMaxDeg = 180;

    public static boolean invertServo = false; //?

    public static double targetX = 40;
    public static double targetY = 10;


    public static double robotHeadingRad = 0;

    private Servo turret;

    @Override
    public void init() {
        turret = hardwareMap.get(Servo.class, "turret");

        // track immediately on init
        aimTurret();
    }

    @Override
    public void loop() {

        aimTurret();

        telemetry.addData("target", "(%.1f, %.1f)", targetX, targetY);
        telemetry.addData("headingDeg", Math.toDegrees(robotHeadingRad));
        telemetry.update();
    }

    private void aimTurret() {
        Vector2d errorVectorField = new Vector2d(targetX, targetY);

        // field angle to target
       double fieldAngle = Math.atan2(errorVectorField.getX(), errorVectorField.getY());

        // robot-relative turret angle
        double turretAngleRad = normalizeRadians(fieldAngle - robotHeadingRad);
        double turretDeg = Math.toDegrees(turretAngleRad);

        // wrap into your preferred range (helps if you use -180..180 style)
        turretDeg = wrapToRangeDeg(turretDeg, turretMinDeg, turretMaxDeg);

        // map degrees -> servo position
        double pos = mapDegToServo(turretDeg);

        turret.setPosition(pos);

        telemetry.addData("turretDeg", turretDeg);
        telemetry.addData("servoPos", pos);
    }

    private double mapDegToServo(double deg) {
        double t = (deg - turretMinDeg) / (turretMaxDeg - turretMinDeg);
        t = clamp(t, 0, 1);
        if (invertServo) t = 1.0 - t;
        return t;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double normalizeRadians(double r) {
        while (r <= -Math.PI) r += 2 * Math.PI;
        while (r > Math.PI) r -= 2 * Math.PI;
        return r;
    }

    private static double wrapToRangeDeg(double deg, double minDeg, double maxDeg) {
        double span = maxDeg - minDeg; // e.g. 360
        while (deg < minDeg) deg += span;
        while (deg > maxDeg) deg -= span;
        // if still outside due to weird min/max, j clamp
        return Math.max(minDeg, Math.min(maxDeg, deg));
    }
}
