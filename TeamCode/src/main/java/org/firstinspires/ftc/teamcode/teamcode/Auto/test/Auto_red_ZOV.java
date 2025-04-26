package org.firstinspires.ftc.teamcode.teamcode.Auto.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.teamcode.PID_setting;
import org.firstinspires.ftc.teamcode.teamcode.openCV.ReducedDetector;
import org.firstinspires.ftc.teamcode.teamcode.openCV.ZID;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(group = "Full auto")
public class Auto_red_ZOV extends LinearOpMode {
    private ZID z = new ZID();
    private ElapsedTime moveTimer = new ElapsedTime();
    private ElapsedTime mover = new ElapsedTime();
    private PID_setting pid_setting = new PID_setting();
    private Driving driving;
    private Count count;

    private DcMotor FL, BL, FR, BR;
    private Servo sbkr, grabr, grabl, sbros;
    private CRServo extl, extr;
    private BNO055IMU Gyro;
    private OpenCvCamera webcam;
    private Telemetry dash;

    // Параметры движения
    private double positiony, positionx;
    private double currentAngle, deltaHedL, deltaHed;
    private double lastposx, lastposy;
    private double Multiply = 0;
    double Angleosui;

    private double turnPower = 0;
    private boolean nottaken = true;
    private double drivePowerx = 0;
    private double drivePowery = 0;
    private double targDistx = 0;
    private double targDisty = 0;
    private double targAngle = 0;
    private double currAngle = 0, AngleL = 0, turnErr = 0, turnErrL = 0;
    private double driveErrx = 0;
    private double driveErry = 0;
    private double driveErrLx = 0;
    private double driveErrLy = 0;
    private double x, y;
    private boolean et = true;
    private double Base = 3;
    private Orientation orientation = new Orientation();
    private double lastAngle;
    private double pos = 0;
    private double lastpos;
    private boolean flag = false;
    private boolean isRunning = true;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initializeHardware();
            initializeCamera();
            waitForStart();

            if (opModeIsActive()) {
                runAutonomous();
            }
        } finally {
            // Гарантированное освобождение ресурсов
            stopAll();
        }
    }

    private void initializeHardware() {
        FL = hardwareMap.dcMotor.get("FL");
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FR = hardwareMap.dcMotor.get("FR");
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BL = hardwareMap.dcMotor.get("BL");
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BR = hardwareMap.dcMotor.get("BR");
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sbkr = hardwareMap.servo.get("sbkr");
        sbros = hardwareMap.servo.get("sbros");
        grabl = hardwareMap.servo.get("grabl");
        grabr = hardwareMap.servo.get("grabr");

        extl = hardwareMap.crservo.get("extl");
        extr = hardwareMap.crservo.get("extr");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        Gyro = hardwareMap.get(BNO055IMU.class, "imu");
        Gyro.initialize(parameters);
    }

    private void initializeCamera() {
        dash = FtcDashboard.getInstance().getTelemetry();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        ReducedDetector detector = new ReducedDetector();
        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        FtcDashboard.getInstance().startCameraStream(webcam, 24);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Base", Base);
            dash.addData("fill", detector.fillValue);
            dash.addData("center", detector.center);
            dash.update();

            if (detector.fillValue < 0.003) {
                Base = 3;
                dash.addData("park", "3");
            } else if (detector.center < 160) {
                Base = 1;
                dash.addData("park", "1");
            } else if (detector.center > 160) {
                Base = 2;
                dash.addData("park", "2");
            }
        }
    }

    private void runAutonomous() throws InterruptedException {
        driving = new Driving();
        count = new Count();

        driving.start();
        count.start();

        dash.addData("angle", currAngle);
        dash.update();

        x = 0;
        y = 0;

        Move(0, 13, 0);
        sleep(1000);

        int park = z.N;
        if ((park == 1) || (park == 2) || (park == 3)) {
            // detector.setHSV(0,0,0,0,0,0); // Раскомментируйте, если detector имеет этот метод
        }

        double fill_needed = 0.02;
        if ((/*detector.fillValue > fill_needed &&*/ nottaken) || park == 1) {
            Move(50, 13, 0);
            nottaken = false;
            grabr.setPosition(0.1);
            grabl.setPosition(0.9);
            sleep(1000);
        } else {
            Move(0, 70, 0);
            sleep(1000);

            if ((/*detector.fillValue > fill_needed &&*/ nottaken) || park == 2) {
                Move(50, 70, 0);
                nottaken = false;
                grabr.setPosition(0.1);
                grabl.setPosition(0.9);
                sleep(1000);
            } else {
                Move(0, 130, 0);
                sleep(1000);
                if ((/*detector.fillValue > fill_needed &&*/ nottaken) || park == 3) {
                    Move(50, 130, 0);
                    nottaken = false;
                    grabr.setPosition(0.1);
                    grabl.setPosition(0.9);
                    sleep(1000);
                }
            }
        }

        webcam.stopStreaming();

        Move(0, 45, 0);
        Move(-60, 45, 0);

        while (FL.getCurrentPosition() < 3900 && opModeIsActive()) {
            dash.addData("posu", FL.getCurrentPosition());
            dash.update();
            extr.setPower(1);
            extl.setPower(1);
        }

        extr.setPower(0);
        extl.setPower(0);

        Mve(-56, 59.58, 90);

        flag = true;
        Mve(-72, 59.58, 90);
        sleep(1000);
        flag = false;

        Move(-65, 40, 180);

        extr.setPower(-1);
        extl.setPower(-1);
        sleep(1500);

        extr.setPower(0);
        extl.setPower(0);

        Mve(-200, 40, 180);

        grabr.setPosition(0.9);
        grabl.setPosition(0);

        sleep(1000);
        Move(-190, 46, 270);
    }

    private void stopAll() {
        isRunning = false;

        // Остановка моторов
        if (FL != null) FL.setPower(0);
        if (FR != null) FR.setPower(0);
        if (BL != null) BL.setPower(0);
        if (BR != null) BR.setPower(0);

        // Остановка сервоприводов
        if (extl != null) extl.setPower(0);
        if (extr != null) extr.setPower(0);

        // Остановка камеры
        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }

        // Остановка потоков
        if (driving != null) driving.interrupt();
        if (count != null) count.interrupt();
    }

    public void Move(double disty, double distx, double turn) {
        targAngle = turn;
        targDisty = disty * 699;
        targDistx = distx * 699;
        moveTimer.reset();

        while (opModeIsActive() &&
                (Math.abs(targDisty - y) > 400 ||
                        Math.abs(targDistx - x) > 400 ||
                        Math.abs(targAngle - currentAngle) > 5)) {
            dash.addData("ignorey", Math.abs(targDisty - y));
            dash.addData("ignorex", Math.abs(targDistx - x));
            dash.addData("ignorea", Math.abs(targAngle - currentAngle));
            dash.update();
        }

        sleep(300);
    }

    public void Mve(double disty, double distx, double turn) {
        targAngle = turn;
        targDisty = disty * 699;
        targDistx = distx * 699;
        moveTimer.reset();
        sleep(2500);
    }

    public double Angle() {
        orientation = Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return orientation.firstAngle;
    }

    class Driving extends Thread {
        @Override
        public void run() {
            while (isRunning && !isInterrupted()) {
                try {
                    double Angle = Angle();
                    double BLs = BL.getCurrentPosition();
                    double BRs = BR.getCurrentPosition();

                    deltaHed = Angle - deltaHedL;
                    if (deltaHed > 180) {
                        deltaHed -= 360;
                    } else if (deltaHed < -180) {
                        deltaHed += 360;
                    }

                    currentAngle += deltaHed;

                    if (currentAngle < 0) {
                        Angleosui = 360 + currentAngle % 360;
                    } else if (currentAngle > 0) {
                        Angleosui = currentAngle % 360;
                    } else {
                        Angleosui = 0;
                    }

                    double deltaAngle = (currentAngle - lastAngle);
                    double deltaY = (BLs - lastposy - (deltaAngle * pid_setting.ignorey));
                    double deltaX = (BRs - lastposx - (deltaAngle * pid_setting.ignorex));

                    double angleRadians = Math.toRadians(Angleosui % 90);
                    double cosAngle = Math.cos(angleRadians);
                    double sinAngle = Math.sin(angleRadians);

                    if (Angleosui >= 270) {
                        y += sinAngle * -deltaY + cosAngle * deltaX;
                        x += -cosAngle * deltaY + sinAngle * -deltaX;
                    } else if (Angleosui >= 180) {
                        y += -cosAngle * -deltaY + sinAngle * deltaX;
                        x += -sinAngle * deltaY - cosAngle * -deltaX;
                    } else if (Angleosui >= 90) {
                        y -= sinAngle * -deltaY + cosAngle * deltaX;
                        x -= -cosAngle * deltaY + sinAngle * -deltaX;
                    } else {
                        y -= -cosAngle * -deltaY + sinAngle * deltaX;
                        x -= -sinAngle * deltaY - cosAngle * -deltaX;
                    }

                    lastposx = BRs;
                    lastposy = BLs;
                    lastAngle = currentAngle;

                    turnErr = targAngle - currentAngle;
                    turnPower = (turnErr * pid_setting.turnKp + (turnErr - turnErrL) * pid_setting.turnKd);
                    driveErry = -targDisty + y;
                    drivePowery = driveErry * pid_setting.driveKp + (driveErry - driveErrLy) * pid_setting.driveKd;
                    driveErrx = targDistx - x;
                    drivePowerx = driveErrx * pid_setting.drivexKp + (driveErrx - driveErrLx) * pid_setting.drivexKd;

                    // Ограничение мощности
                    drivePowerx = Math.max(-0.6, Math.min(0.6, drivePowerx));
                    drivePowery = Math.max(-0.6, Math.min(0.6, drivePowery));
                    turnPower = Math.max(-0.8, Math.min(0.8, turnPower));

                    double power = Math.sqrt((drivePowerx * drivePowerx) + (drivePowery * drivePowery));
                    double radian = Math.atan2(drivePowery, drivePowerx);

                    FR.setPower(((power * Math.cos(radian - Math.PI / 4 + Math.toRadians(currentAngle) + Math.PI) * Math.sqrt(2)) + turnPower) * -Multiply);
                    FL.setPower(((power * Math.cos(radian - 3 * Math.PI / 4 + Math.toRadians(currentAngle) + Math.PI) * Math.sqrt(2)) - turnPower) * Multiply);
                    BR.setPower(((power * Math.cos(radian - 3 * Math.PI / 4 + Math.toRadians(currentAngle) + Math.PI) * Math.sqrt(2)) + turnPower) * -Multiply);
                    BL.setPower(((power * Math.cos(radian - Math.PI / 4 + Math.toRadians(currentAngle) + Math.PI) * Math.sqrt(2)) - turnPower) * Multiply);

                    turnErrL = turnErr;
                    driveErrLy = driveErry;
                    driveErrLx = driveErrx;
                    AngleL = currAngle;

                    dash.addData("x", x / 699);
                    dash.addData("y", y / 699);
                    dash.addData("A", currentAngle);
                    dash.update();

                    deltaHedL = Angle;

                    if (lastpos != pos) {
                        moveTimer.reset();
                    }
                    lastpos = pos;

                    if (flag && et) {
                        if (pos == 2) {
                            if (moveTimer.milliseconds() < 500) {
                                sbros.setPosition(0.5);
                            } else {
                                pos = 1;
                            }
                        } else {
                            if (moveTimer.milliseconds() < 500) {
                                sbros.setPosition(1);
                            } else {
                                pos = 2;
                            }
                        }
                    } else {
                        sbros.setPosition(0);
                    }
                } catch (Exception e) {
                    telemetry.addData("Driving Thread Error", e.getMessage());
                    telemetry.update();
                    break;
                }
            }

            // Остановка моторов при завершении потока
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }
    }

    class Count extends Thread {
        @Override
        public void run() {
            boolean sbros = false;
            boolean down = false;
            double count = 0;
            boolean mover_timer_reseted = false;

            while (isRunning) {
                try {
                    if (flag && !(mover_timer_reseted)) {
                        mover.reset();
                        mover_timer_reseted = true;
                    }

                    if (flag && mover.milliseconds() < 3000) {
                        Multiply = 0.4;

                        int redsbros = colorSensorSbros.red();
                        int greensbros = colorSensorSbros.green();
                        int bluesbros = colorSensorSbros.blue();

                        if (redsbros > greensbros && redsbros > bluesbros) {
                            sbros = true;
                        }

                        telemetry.addData("redsbros", redsbros);
                        telemetry.addData("gren", greensbros);
                        telemetry.addData("blue", bluesbros);
                        telemetry.addData("me", mover.milliseconds());
                        telemetry.update();

                        if (sbros) {
                            count += 0.5;
                            sbros = false;

                            while (((colorSensordown.red() < colorSensordown.green()) ||
                                    (colorSensordown.red() < colorSensordown.blue())) &&
                                    mover.milliseconds() < 4000 &&
                                    count < 2
                                    ) {
                                Multiply = 0;
                                et = false;
                            }

                            et = true;
                            dash.addData("count", count);
                            dash.update();
                        }
                    } else if (flag) {
                        Multiply = 1.5;
                        et = false;
                    } else {
                        Multiply = 1;
                    }
                } catch (Exception e) {
                    telemetry.addData("Count Thread Error", e.getMessage());
                    telemetry.update();
                    break;
                }
            }
        }
    }

    private static class colorSensorSbros {
        static int red() {
            return 0;
        }

        static int green() {
            return 0;
        }

        static int blue() {
            return 0;
        }
    }

    private static class colorSensordown {
        static int red() {
            return 0;
        }

        static int green() {
            return 0;
        }

        static int blue() {
            return 0;
        }
    }
}