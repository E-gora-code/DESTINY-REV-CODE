package teamcode.teamcode.RobotModules.turret;

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
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class LimelightTurret extends LinearOpMode {

    private Limelight3A limelight;
    private CRServo servoYaw;

    private DcMotorEx shooterLeft, shooterRight;
    private IMU imu;

    private final double CENTER_PITCH = 0.5;
    private final double PITCH_RANGE_DEG = 180.0;

    private double filteredDistance = 0;
    private double filteredTx = 0; // Keep tx for yaw control
    private double filteredTy = 0; // Add ty filtering
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
        shooterLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);





        limelight.pipelineSwitch(5);
        limelight.start();

        waitForStart();
        limelight.updateRobotOrientation(0);

        while (opModeIsActive()) {

            long now = System.currentTimeMillis();
            double dt = (lastTime == 0) ? 0.02 : (now - lastTime) / 1000.0;
            lastTime = now;

            LLResult result = limelight.getLatestResult();

            double tx = 0, ty = 0;
            double distanceCm = 0;
            double xCm = 110, yCm = 110, zCm = 110;
            double roll = 0, pitch = 0, yaw = 0;
            double targetRoll = 0, targetPitch = 0, targetYaw = 0;
            boolean hasTarget = false;

            if (result != null && result.isValid()) {
                hasTarget = true;

                // Get horizontal and vertical angles (in degrees)
                tx = result.getTx();  // Horizontal offset in degrees
                ty = result.getTy();  // Vertical offset in degrees

                // Get 3D position if available (from botpose)
                Pose3D botpose = result.getBotpose();
                Pose3D botposeMT2 = result.getBotpose();



                if (botposeMT2 != null) {

                    Position pos = botposeMT2.getPosition();
                    YawPitchRollAngles orient = botposeMT2.getOrientation();
                    xCm = pos.x * 100;
                    yCm = pos.y * 100;
                    zCm = pos.z * 100;

                    // Get orientation in degrees
                    yaw = Math.toDegrees(orient.getYaw());
                    pitch = Math.toDegrees(orient.getPitch());
                    roll = Math.toDegrees(orient.getRoll());

                    distanceCm = Math.sqrt(xCm * xCm + yCm * yCm);

                }

                // Get target pose in camera space
                
            }

            // Apply filters
            filteredTx = FILTER_A * tx + (1 - FILTER_A) * filteredTx;
            filteredTy = FILTER_A * ty + (1 - FILTER_A) * filteredTy;
            filteredDistance = FILTER_A * distanceCm + (1 - FILTER_A) * filteredDistance;

            // Dashboard output
            dash.addData("=== TARGET INFO ===", "");
            dash.addData("Has Target", hasTarget);
            dash.addData("TX (deg)", String.format("%.2f", tx));
            dash.addData("TY (deg)", String.format("%.2f", ty));
            dash.addData("Filtered TX (deg)", String.format("%.2f", filteredTx));
            dash.addData("Filtered TY (deg)", String.format("%.2f", filteredTy));

            dash.addData("=== POSITION (cm) ===", "");
            dash.addData("X (forward)", String.format("%.1f", xCm));
            dash.addData("Y (right)", String.format("%.1f", yCm));
            dash.addData("Z (up)", String.format("%.1f", zCm));
            dash.addData("Distance XY", String.format("%.1f", distanceCm));
            dash.addData("Filtered Distance", String.format("%.1f", filteredDistance));

            dash.addData("=== ROBOT ORIENTATION (deg) ===", "");
            dash.addData("Roll", String.format("%.1f", roll));
            dash.addData("Pitch", String.format("%.1f", pitch));
            dash.addData("Yaw", String.format("%.1f", yaw));

            dash.addData("=== TARGET ORIENTATION (deg) ===", "");
            dash.addData("Target Roll", String.format("%.1f", targetRoll));
            dash.addData("Target Pitch", String.format("%.1f", targetPitch));
            dash.addData("Target Yaw", String.format("%.1f", targetYaw));

            dash.update();

            double yawPower = 0.0;

            if (hasTarget) {
                // Use tx for yaw control (horizontal offset in degrees)
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






            boolean shoot = true;

            double desiredRpm;
            if (shoot && hasTarget && filteredDistance > 0) {
                // Adjust RPM based on distance (in cm)
                desiredRpm = 260 + (filteredDistance / 100.0) * 180;
            } else if (shoot) {
                desiredRpm = 260; // Default value
            } else {
                desiredRpm = 0;
            }

            if (desiredRpm > SHOOT_TARGET_RPM) desiredRpm = SHOOT_TARGET_RPM;
            shooterTargetRpm = desiredRpm;


            double vel = shooterRight.getVelocity();

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


            if (shooterTargetRpm <= 0) power = 0;
            power = clamp(power,0,1);


            shooterLeft.setPower(power);
            shooterRight.setPower(power);

            // Telemetry output
            telemetry.addData("=== TARGET INFO ===", "");
            telemetry.addData("Has Target", hasTarget);
            telemetry.addData("TX (deg)", String.format("%.2f", tx));
            telemetry.addData("TY (deg)", String.format("%.2f", ty));

            telemetry.addData("=== POSITION (cm) ===", "");
            telemetry.addData("X (forward)", String.format("%.1f", xCm));
            telemetry.addData("Y (right)", String.format("%.1f", yCm));
            telemetry.addData("Z (up)", String.format("%.1f", zCm));
            telemetry.addData("Distance", String.format("%.1f cm", distanceCm));
            telemetry.addData("Filtered Distance", String.format("%.1f cm", filteredDistance));

            telemetry.addData("=== ANGLES (deg) ===", "");


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