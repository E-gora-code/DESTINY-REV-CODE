package org.firstinspires.ftc.teamcode.turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp
public class LimelightTurret extends LinearOpMode {

    private Limelight3A limelight;
    private CRServo servoYaw;
    private Servo servoPitch;
    private DcMotorEx shooterLeft, shooterRight;
    private IMU imu;

    private final double CENTER_PITCH = 0.5;
    private final double PITCH_RANGE_DEG = 180.0;

    private double filteredTx = 0, filteredTy = 0;
    private final double FILTER_A = 0.35;

    private long lastTime = 0;

    private static final int TICKS_PER_REV = 560;
    private final double SHOOT_TARGET_RPM = 280.0;
    private final double RAMP_TIME = 0.25;


    private SmoothPID yawPID = new SmoothPID(0.0035, 0.0001, 0.0005, 0.23);


    private SmoothPID pitchPID = new SmoothPID(0.01, 0.00005, 0.001, 0.02);


    private double shooterKp = 0.0022;
    private double shooterKi = 0.0;
    private double shooterKd = 0.00045;
    private double shooterKf = 1.0 / 300.0;
    private double shooterIntegral = 0.0;
    private double shooterPrevError = 0.0;
    private double shooterTargetRpm = 0.0;

    private final double GRAVITY = 9.81;
    private final double CAMERA_HEIGHT_M = 0.25;
    private final double TARGET_HEIGHT_M = 0.40;
    private double shooterVelocityMps = 10.0;
    Telemetry dash = FtcDashboard.getInstance().getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        servoYaw = hardwareMap.get(CRServo.class, "servoX");
        servoPitch = hardwareMap.get(Servo.class, "servoY");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        shooterRight.setDirection(DcMotorEx.Direction.REVERSE);
        shooterLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        shooterLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        servoPitch.setPosition(CENTER_PITCH);

        limelight.pipelineSwitch(5);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            long now = System.currentTimeMillis();
            double dt = (lastTime == 0) ? 0.02 : (now - lastTime) / 1000.0;
            lastTime = now;

            LLResult result = limelight.getLatestResult();

            double tx = 0, ty = 0;
            double distanceM = Double.NaN;
            boolean hasTarget = false;

            if (result != null && result.isValid()) {
                hasTarget = true;
                tx = result.getTx();
                ty = result.getTy();
                Pose3D pose = result.getBotpose_MT2();
                if (pose == null) pose = result.getBotpose();
                if (pose != null) {
                    double x = pose.getPosition().x;
                    double y = pose.getPosition().y;
                    distanceM = Math.sqrt(x * x + y * y);
                }
            }

            filteredTx = FILTER_A * tx + (1 - FILTER_A) * filteredTx;
            filteredTy = FILTER_A * ty + (1 - FILTER_A) * filteredTy;
            dash.addData("tx",filteredTx);
            dash.update();

            double yawPower = 0.0;

            if (hasTarget) {

                double yawError = filteredTx;
                double deadzone = 0.5;
                if (Math.abs(yawError) < deadzone) {
                    yawPower = 0;
                } else {
                    yawPower = yawPID.update(yawError, dt);
                }
            } else {
                yawPower = 0;
                yawPID.resetIntegral();
            }

            servoYaw.setPower(yawPower);
            double pitchPos = servoPitch.getPosition();
            double currPitchDeg = (pitchPos - CENTER_PITCH) * PITCH_RANGE_DEG;

            double pitchTargetDeg = currPitchDeg;

            if (hasTarget && !Double.isNaN(distanceM)) {
                double h = TARGET_HEIGHT_M - CAMERA_HEIGHT_M;
                double term = (GRAVITY * distanceM) / (shooterVelocityMps * shooterVelocityMps);
                if (Math.abs(term) <= 1.0) {
                    double base = Math.toDegrees(0.5 * Math.asin(term));
                    double corr = Math.toDegrees(Math.atan2(h, distanceM));
                    pitchTargetDeg = base + corr;
                }
            }

            double pitchError = pitchTargetDeg - currPitchDeg;
            double pitchOut = pitchPID.update(pitchError, dt);

            double newPitchDeg = currPitchDeg + pitchOut;
            if (newPitchDeg > 60) newPitchDeg = 60;
            if (newPitchDeg < -10) newPitchDeg = -10;

            double newPos = CENTER_PITCH + newPitchDeg / PITCH_RANGE_DEG;
            newPos = clamp(newPos, 0.0, 1.0);
            servoPitch.setPosition(newPos);

            // Управление стреляющим механизмом (остается без изменений)
            boolean shoot = gamepad1.right_bumper;

            double desiredRpm;
            if (shoot && hasTarget && !Double.isNaN(distanceM)) {
                desiredRpm = 260 + distanceM * 180;
            } else if (shoot) {
                desiredRpm = 220;
            } else {
                desiredRpm = 0;
            }

            if (desiredRpm > SHOOT_TARGET_RPM) desiredRpm = SHOOT_TARGET_RPM;

            double alpha = Math.min(1.0, dt / RAMP_TIME);
            shooterTargetRpm += (desiredRpm - shooterTargetRpm) * alpha;

            double vel = shooterLeft.getVelocity();
            double currentRpm = vel * 60.0 / TICKS_PER_REV;

            double err = shooterTargetRpm - currentRpm;
            shooterIntegral += err * dt;
            shooterIntegral = clamp(shooterIntegral, -4000, 4000);
            double der = (err - shooterPrevError) / Math.max(dt, 1e-6);
            shooterPrevError = err;

            double power = shooterKf * shooterTargetRpm
                    + shooterKp * err
                    + shooterKi * shooterIntegral
                    + shooterKd * der;

            power = clamp(power, 0.0, 1.0);
            if (shooterTargetRpm <= 0) power = 0;

            shooterLeft.setPower(power);
            shooterRight.setPower(power);

            telemetry.addData("HasTag", hasTarget);
            telemetry.addData("tx", tx);
            telemetry.addData("FilteredTx", filteredTx);
            telemetry.addData("YawPower", yawPower);
            telemetry.addData("YawPID Error", hasTarget ? filteredTx : 0);
            telemetry.addData("PitchPos", servoPitch.getPosition());
            telemetry.addData("RPM", currentRpm);
            telemetry.update();
        }

        limelight.stop();
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public static class SmoothPID {
        private double kp, ki, kd;
        private double integral = 0, prevError = 0, prevOut = 0;
        private double limit = 1.0;
        private final double smooth = 0.18;

        public SmoothPID(double kp, double ki, double kd, double limit) {
            this.kp = kp; this.ki = ki; this.kd = kd; this.limit = limit;
        }

        public double update(double e, double dt) {
            integral += e * dt;
            integral = Math.max(-500, Math.min(500, integral));
            double d = (e - prevError) / Math.max(dt, 1e-6);
            prevError = e;
            double raw = kp * e + ki * integral + kd * d;
            raw = Math.max(-limit, Math.min(limit, raw));
            double out = prevOut + (raw - prevOut) * smooth;
            prevOut = out;
            return out;
        }

        public void resetIntegral() {
            integral = 0;
        }

        public void setGains(double kp, double ki, double kd) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
        }
    }
}