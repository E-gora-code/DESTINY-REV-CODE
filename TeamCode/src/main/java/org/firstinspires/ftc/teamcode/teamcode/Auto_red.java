package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
public class Auto_red extends LinearOpMode {

    ElapsedTime moveTimer = new ElapsedTime();
    ElapsedTime mover = new ElapsedTime();
    PID_setting pid_setting = new PID_setting();
    Driving driving = new Driving();
    Count count = new Count();

    DcMotor FL, BL, FR, BR;
    ColorSensor colorSensorSbros,colorSensordown;
    Servo sbkr, grabr, grabl,sbros;
    double positiony,positionx;
    double Angleosui;
    double currentAngle,deltaHedL,deltaHed;
    double lastposx,lastposy;
    double Multiply = 0;
    double turnPower = 0;
    boolean nottaken = true;
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
    boolean et= true;
    CRServo extl,extr;
    double Base = 3;
    BNO055IMU Gyro;
    Orientation Orientation = new Orientation();
    double lastAngle;
    OpenCvCamera webcam;
    Telemetry dash = FtcDashboard.getInstance().getTelemetry();
    double pos = 0;
    double lastpos;
    boolean flag = false;

    @Override
    public void runOpMode() throws InterruptedException {
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
        colorSensorSbros = hardwareMap.get(ColorSensor.class, "color_sensor");
        colorSensordown = hardwareMap.get(ColorSensor.class, "color_sensor1");



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
        while (!opModeIsActive()&&!isStopRequested()) {
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
        count.start();
//        while (opModeIsActive()) {
        dash.addData("angle", currAngle);
        dash.update();
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        Gyro = hardwareMap.get(BNO055IMU.class, "imu");
        Gyro.initialize(parameters);

        x=0;
        y=0;



        Move(0, 13, 0);
        if(!isStopRequested()){
            sleep(500);
        }




        if (detector.fillValue > 0.02 && nottaken) {
            Move(50,13,0);
            nottaken = false;
            grabr.setPosition(0.1);
            grabl.setPosition(0.9);


            sleep(1000);

        }
        else {
            Move(0, 70, 0);
            if(!isStopRequested()){
                sleep(500);
            }


            if (detector.fillValue > 0.02 && nottaken) {
                Move(50, 70, 0);
                nottaken = false;
                grabr.setPosition(0.1);
                grabl.setPosition(0.9);


                sleep(1000);

            }
            else{
                Move(0, 130, 0);
                if(!isStopRequested()){
                    sleep(500);
                }
                if (detector.fillValue > 0.02 && nottaken) {
                    Move(50, 130, 0);
                    nottaken = false;
                    grabr.setPosition(0.1);
                    grabl.setPosition(0.9);
                    sleep(1000);
            }
            }
        }
        webcam.stopStreaming();


        Move(0,45,0);
        Move(-40,45,0);
        while (FL.getCurrentPosition()<3900&&!isStopRequested()){
            dash.addData("posu",FL.getCurrentPosition());
            dash.update();
            extr.setPower(-1);
            extl.setPower(-1);
        }
        extr.setPower(0);
        extl.setPower(0);

        Move(-36,59.58,90);



        //git

        flag = true;
        Move(-52,59.58,90);

        flag = false;


        Move(-85,50,0);

        extr.setPower(1);
        extl.setPower(1);
        sleep(1500);

        extr.setPower(0);
        extl.setPower(0);

        Move(0,45,0);
        Move(0, 130, 0);




        Mve(-220,125,0);


        Move(-220,120,90);

        grabr.setPosition(0.9);
        grabl.setPosition(0);








        sleep(1000);
        Move(-70,110,-90);




    }

    public void Move(double disty,double distx,double turn) {

        targAngle = turn;
        targDisty = disty * 699;
        targDistx = distx * 699;
        moveTimer.reset();
        while ((Math.abs(targDisty-y)>400 || Math.abs(targDistx-x)>400 || Math.abs(targAngle-currentAngle)>5)&&!isStopRequested()) {
            dash.addData("ignorey",Math.abs(targDisty-y));
            dash.addData("ignorex",Math.abs(targDistx-x));
            dash.addData("ignorea",Math.abs(targAngle-currentAngle));
            dash.update();
        }
        if (!isStopRequested()) {
            sleep(300);
            dash.addData("ignorey", Math.abs(targDisty - y) > 500);
            dash.addData("ignorex", Math.abs(targDistx - x) > 500);
            dash.addData("ignorea", Math.abs(targAngle - currentAngle) > 5);

            dash.update();
        }
    }
    public void Mve(double disty,double distx,double turn) {

        targAngle = -turn;
        targDisty = disty * 699;
        targDistx = distx * 699;
        moveTimer.reset();

        sleep(3000);

    }


    public double Angle() {
        Orientation = Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Orientation.firstAngle;
    }

    class Driving extends Thread {
        public void run() {
            while (opModeIsActive()&&!isStopRequested()) {
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

// Обновление последних позиций
                lastposx = BRs;
                lastposy = BLs;
                lastAngle = currentAngle;




                turnErr = targAngle - currentAngle;
                turnPower = (turnErr * pid_setting.turnKp + (turnErr - turnErrL) * pid_setting.turnKd);
                driveErry =-targDisty + y;
                drivePowery = driveErry * pid_setting.driveKp + (driveErry - driveErrLy) * pid_setting.driveKd;
                driveErrx = targDistx - x;
                drivePowerx = driveErrx * pid_setting.drivexKp + (driveErrx - driveErrLx) * pid_setting.drivexKd;
                if (drivePowerx > 0.7) {
                    drivePowerx = 0.6;
                }
                if (drivePowerx < -0.7) {
                    drivePowerx = -0.6;
                }
                if (drivePowery > 0.7) {
                    drivePowery = 0.6;
                }
                if (drivePowery < -0.7) {
                    drivePowery = -0.6;
                }
                if (turnPower > 0.5) {
                    turnPower = 0.8;
                }
                if (turnPower < -0.5) {
                    turnPower = -0.8;
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
                dash.addData("x",x/699);
                dash.addData("y",y/699);
                dash.addData("A",currentAngle);
                dash.update();






                deltaHedL = Angle;
                if (lastpos!=pos){
                    moveTimer.reset();

                }
                lastpos = pos;

                if (flag && et){
                    if (pos ==2){
                        if (moveTimer.milliseconds() <500){
                            sbros.setPosition(0.5);
                        }
                        else {
                            pos = 1;
                        }
                    }
                    else{
                        if (moveTimer.milliseconds() <500){
                            sbros.setPosition(1);
                        }
                        else {
                            pos = 2;
                        }
                    }

                }
                else{
                    sbros.setPosition(0);

                }



            }
        }
    }
    class Count extends Thread {
        public void run() {
            boolean sbros = false;
            boolean down = false;
            double count = 0;

            boolean mover_timer_reseted = false;

            while (opModeIsActive()&&!isStopRequested()) {
                if (flag && !(mover_timer_reseted)){
                    mover.reset();
                    mover_timer_reseted = true;
                }
                if (flag && mover.milliseconds()<3000){
                    Multiply = 0.45;

                    int redsbros = colorSensorSbros.red();
                    int greensbros = colorSensorSbros.green();
                    int bluesbros = colorSensorSbros.blue();
                    if (redsbros>greensbros &&redsbros>bluesbros){
                        sbros = true;
                    }
                    telemetry.addData("redsbros",redsbros);
                    telemetry.addData("gren",greensbros);
                    telemetry.addData("blue",bluesbros);
                    telemetry.addData("me",mover.milliseconds());

                    telemetry.update();

                    if (sbros){
                        count += 0.5;
                        sbros = false;
                        while (((colorSensordown.red()<colorSensordown.green()) || (colorSensordown.red()<colorSensordown.blue()))&& mover.milliseconds()<4000 && count <2&&!isStopRequested()){
                            Multiply = 0;
                            et = false;

                        }
                        et = true;
                        dash.addData("count",count);
                        dash.update();
                    }


                }
                else if(flag){
                    Multiply =1.7;

                    et = false;
                }
                else{
                    Multiply = 1;
                }
            }
        }
    }
}


