package org.firstinspires.ftc.teamcode.turret;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class turet {
    private Limelight3A limelight;
    private CRServo yaw;
    private Servo pitch;
    private DcMotorEx shooterL, shooterR;

    private double filtTx = 0, filtTy = 0;
    private double targetRpm = 0;

    public turet(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, "limelight");
        yaw = hw.get(CRServo.class, "servoX");
        pitch = hw.get(Servo.class, "servoY");
        shooterL = hw.get(DcMotorEx.class, "shooterLeft");
        shooterR = hw.get(DcMotorEx.class, "shooterRight");

        shooterR.setDirection(DcMotorEx.Direction.REVERSE);
        shooterL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void reset() {
        pitch.setPosition(0.5);
        limelight.start();
    }

    public void update(boolean shoot) {
        LLResult r = limelight.getLatestResult();
        if (r != null && r.isValid()) {
            double tx = r.getTx();
            double ty = r.getTy();
            filtTx = 0.35 * tx + 0.65 * filtTx;
            filtTy = 0.35 * ty + 0.65 * filtTy;

            double yawPower = pid(filtTx, 0, 0.0035, 0.0001, 0.0005, 0.02, -0.23, 0.23, 0.18, 500);
            yaw.setPower(Math.abs(filtTx) < 0.5 ? 0 : yawPower);


            double yDist = filtTy * 8.485;
            if (yDist > 172.8) yDist = Math.max(345.6 - yDist,0);

            double angle = 50-yDist *0.089;
            angle = clamp(angle, 30, 50);

            double sinA = Math.sin(Math.toRadians(angle));
            double cosA = Math.cos(Math.toRadians(angle));
            double denom = 2 * (yDist * sinA - 8.485 * cosA) * cosA;

            if (denom > 0) {
                double vel = Math.sqrt((9.8 * yDist * yDist) / denom);
                targetRpm = vel * 60.0 / (2 * Math.PI * 0.05);
                targetRpm = Math.min(targetRpm, 6000.0);
            }

            double pitchPos = 0.5 + angle / 180;
            pitchPos = clamp(pitchPos, 0.0, 1.0);
            pitch.setPosition(pitchPos);


            double currRpm = shooterL.getVelocity() * 60.0 / 560;
            double shooterOut = pid(targetRpm, currRpm, 0.0022, 0, 0.00045, 0.02, -1, 1, 0.5, 4000);
            double power = (1.0/300) * targetRpm + shooterOut;
            power = clamp(power, 0.0, 1.0);
            if (targetRpm <= 0) power = 0;

            shooterL.setPower(power);
            shooterR.setPower(power);
        }
    }

    private double pid(double target, double current, double kp, double ki, double kd,
                       double dt, double minOut, double maxOut, double smooth, double iLimit) {
        double integral = 0;
        double prevError = 0;
        double prevOut = 0;

        double error = target - current;
        integral += error * dt;
        integral = clamp(integral, -iLimit, iLimit);
        double deriv = (error - prevError) / Math.max(dt, 0.001);
        prevError = error;

        double raw = kp * error + ki * integral + kd * deriv;
        raw = clamp(raw, minOut, maxOut);

        double output = prevOut + (raw - prevOut) * smooth;
        prevOut = output;

        return output;
    }

    public boolean ready() {
        double currRpm = shooterL.getVelocity() * 60.0 / 560;
        double ratio = targetRpm > 0 ? currRpm / targetRpm : 0;
        return targetRpm > 10 && ratio > 0.95 && Math.abs(filtTx) < 0.5;
    }

    public void stop() {
        yaw.setPower(0);
        shooterL.setPower(0);
        shooterR.setPower(0);
        limelight.stop();
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}