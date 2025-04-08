package org.firstinspires.ftc.teamcode.teamcode;

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
import org.firstinspires.ftc.teamcode.teamcode.openCV.Detector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class Bro_drive extends LinearOpMode {

    ElapsedTime moveTimer = new ElapsedTime();
    PID_setting pid_setting = new PID_setting();
    Driving driving = new Driving();
    DcMotor FL, BL, FR, BR;
    Servo sbkr, grabr, grabl,sbros;
    double positiony,positionx;
    double Angleosui;
    double pos;
    double dobangle;
    double currentAngle,deltaHedL,deltaHed;
    double lastposx,lastposy;
    double Multiply = 1;
    double turnPower = 0;
    double drivePowerx= 0;
    double drivePowery = 0;
    double targDistx = 0;
    double targDisty = 0;
    double targAngle = 0;
    double currAngle = 0, AngleL = 0, turnErr = 0, turnErrL = 0;
    double driveErrx = 0;
    double driveErry = 0;
    double driveErrLx = 0;
    double driveErrLy= 0;
    double x,y;
    CRServo extl,extr;
    double Base = 3;
    BNO055IMU Gyro;
    Orientation Orientation = new Orientation();
    double lastAngle;
    OpenCvCamera webcam;
    Telemetry dash = FtcDashboard.getInstance().getTelemetry();


    @Override
    public void runOpMode() throws InterruptedException {
        FL = hardwareMap.dcMotor.get("FL");
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR = hardwareMap.dcMotor.get("FR");
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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


//        s = hardwareMap.servo.get("servo3");
//        s1=hardwareMap.servo.get("servo2");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        Gyro = hardwareMap.get(BNO055IMU.class, "imu");
        Gyro.initialize(parameters);
        telemetry.addLine("Ready");
        telemetry.update();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        Detector detector = new Detector();
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        FtcDashboard.getInstance().startCameraStream(webcam, 24);
        while (!opModeIsActive()) {
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

        waitForStart();


//        webcam.stopStreaming();
        driving.start();
//        while (opModeIsActive()) {
        dash.addData("angle", currAngle);
        dash.update();
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        Gyro = hardwareMap.get(BNO055IMU.class, "imu");
        Gyro.initialize(parameters);
        while (opModeIsActive()) {
            Move(0, 30, 0, 2000);
            sleep(2000);
            Move(0, 0, 0, 2000);
            sleep(2000);
        }
        driving.stop();


















    }

    public void Move(double disty,double distx,double turn,int Milliseconds) {

        targAngle = turn;
        targDisty = disty * 115;
        targDistx = distx * 512;
        moveTimer.reset();
        while (moveTimer.milliseconds() < Milliseconds) {


        }

    }

    public void Turn(double turn, int Milliseconds) {
        targAngle = turn;
        moveTimer.reset();
        drivePowerx = 0;
        drivePowery = 0;
        targDistx = BR.getCurrentPosition();
        targDisty=BL.getCurrentPosition();
        while (moveTimer.milliseconds() < Milliseconds) {
            drivePowerx = 0;
            drivePowery = 0;
            targDisty=BL.getCurrentPosition();
            targDistx = BR.getCurrentPosition();
        }
    }

    public double Angle() {
        Orientation = Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Orientation.firstAngle;
    }

    class Driving extends Thread {
        public void run() {
            while (opModeIsActive()) {
                double Angle = Angle();

                deltaHed = Angle-deltaHedL;
                if(deltaHed>180){
                    deltaHed -= 360;
                }
                else if(deltaHed<-180){
                    deltaHed += 360;
                }
                currentAngle += deltaHed;
                if (currentAngle >= 0){
                    Angleosui = 360 -currentAngle%360;
                }
                else{
                    Angleosui = -currentAngle%360;
                }




                if (Angleosui>=270){
                    y+=Math.sin(Math.toRadians(Angleosui%90))*(BL.getCurrentPosition()-lastposy-(currentAngle-lastAngle)*pid_setting.ignorey)+Math.cos(Math.toRadians(Angleosui%90))*(BR.getCurrentPosition()-lastposx-(currentAngle-lastAngle)*pid_setting.ignorex);
                    x+=-Math.cos(Math.toRadians(Angleosui%90))*(BL.getCurrentPosition()-lastposy-(currentAngle-lastAngle)*pid_setting.ignorey)+Math.sin(Math.toRadians(Angleosui%90))*(BR.getCurrentPosition()-lastposx-(currentAngle-lastAngle)*pid_setting.ignorex);
                } else if (Angleosui>=180) {
                    y+=-Math.cos(Math.toRadians(Angleosui%90))*(BL.getCurrentPosition()-lastposy-(currentAngle-lastAngle)*pid_setting.ignorey)+Math.sin(Math.toRadians(Angleosui%90))*(BR.getCurrentPosition()-lastposx-(currentAngle-lastAngle)*pid_setting.ignorex);
                    x+=-Math.sin(Math.toRadians(Angleosui%90))*(BL.getCurrentPosition()-lastposy-(currentAngle-lastAngle)*pid_setting.ignorey)-Math.cos(Math.toRadians(Angleosui%90))*(BR.getCurrentPosition()-lastposx-(currentAngle-lastAngle)*pid_setting.ignorex);
                } else if (Angleosui>=90) {
                    y-=Math.sin(Math.toRadians(Angleosui%90))*(BL.getCurrentPosition()-lastposy-(currentAngle-lastAngle)*pid_setting.ignorey)+Math.cos(Math.toRadians(Angleosui%90))*(BR.getCurrentPosition()-lastposx-(currentAngle-lastAngle)*pid_setting.ignorex);
                    x-=-Math.cos(Math.toRadians(Angleosui%90))*(BL.getCurrentPosition()-lastposy-(currentAngle-lastAngle)*pid_setting.ignorey)+Math.sin(Math.toRadians(Angleosui%90))*(BR.getCurrentPosition()-lastposx-(currentAngle-lastAngle)*pid_setting.ignorex);
                }else {
                    y-=-Math.cos(Math.toRadians(Angleosui%90))*(BL.getCurrentPosition()-lastposy-(currentAngle-lastAngle)*pid_setting.ignorey)+Math.sin(Math.toRadians(Angleosui%90))*(BR.getCurrentPosition()-lastposx-(currentAngle-lastAngle)*pid_setting.ignorex);
                    x-=-Math.sin(Math.toRadians(Angleosui%90))*(BL.getCurrentPosition()-lastposy-(currentAngle-lastAngle)*pid_setting.ignorey)-Math.cos(Math.toRadians(Angleosui%90))*(BR.getCurrentPosition()-lastposx-(currentAngle-lastAngle)*pid_setting.ignorex);
                }



                turnErr = targAngle - currentAngle;
                turnPower = (turnErr * pid_setting.turnKp + (turnErr - turnErrL) * pid_setting.turnKd)*0.2;
                driveErry =targDisty - y;
                drivePowery = driveErry * pid_setting.driveKp + (driveErry - driveErrLy) * pid_setting.driveKd;
                driveErrx = targDistx + x;
                drivePowerx = driveErrx * pid_setting.driveKp + (driveErrx - driveErrLx) * pid_setting.driveKd;
                if (drivePowerx > 0.7) {
                    drivePowerx = 0.7;
                }
                if (drivePowerx < -0.7) {
                    drivePowerx = -0.7;
                }
                if (drivePowery > 0.7) {
                    drivePowery = 0.7;
                }
                if (drivePowery < -0.7) {
                    drivePowery = -0.7;
                }
                if (turnPower > 0.7) {
                    turnPower = 0.7;
                }
                if (turnPower < -0.7) {
                    turnPower = -0.7;
                }
                double power = Math.sqrt((drivePowerx*drivePowerx)+(drivePowery*drivePowery));
                double radian = Math.atan2(drivePowery,drivePowerx);


                FR.setPower(((power*Math.cos(radian-Math.PI/4+Math.toRadians(currentAngle)+Math.PI)*Math.sqrt(2))+turnPower)*-Multiply);
                FL.setPower(((power*Math.cos(radian-3*Math.PI/4+Math.toRadians(currentAngle)+Math.PI)*Math.sqrt(2))-turnPower)*Multiply);
                BR.setPower(((power*Math.cos(radian-3*Math.PI/4+Math.toRadians(currentAngle)+Math.PI)*Math.sqrt(2))+turnPower)*-Multiply);
                BL.setPower(((power*Math.cos(radian-Math.PI/4+Math.toRadians(currentAngle)+Math.PI)*Math.sqrt(2))-turnPower)*Multiply);



                turnErrL = turnErr;
                driveErrLy = driveErry;
                driveErrLx = driveErrx;
                AngleL = currAngle;
                telemetry.addData("angle", currentAngle);
                telemetry.addData("error", turnErr);
                telemetry.update();
                telemetry.addData("Base", Base);
                telemetry.addData("positionx",x/115);
                telemetry.addData("positiony",y/115);
                dash.addData("targAngle",currentAngle);
                dash.addData("positionx",x);
                dash.addData("positiony",y/115);
                dash.update();
                lastposx = BR.getCurrentPosition();
                lastposy = BL.getCurrentPosition();
                lastAngle = currentAngle;
                deltaHedL = Angle;
            }
        }
    }
}

