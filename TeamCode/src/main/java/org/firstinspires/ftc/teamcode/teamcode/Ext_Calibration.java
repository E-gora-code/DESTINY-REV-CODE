package org.firstinspires.ftc.teamcode.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.TreeMap;


//@Autonomous
public class Ext_Calibration extends LinearOpMode {
                                                                                                                                                                                                                                                                                            boolean xkfi = false;
    PID_setting pid_setting = new PID_setting();
    Extention Exten = new Extention();
    // config
    boolean simple_ext = false;

    // end
    double mult = 1;
    double extr_zero = 0, extl_zero = 0;
    double extr_max = 5588, extl_max = 3717;
    double ext_range = 5000;
    DcMotor FL, BL, FR, BR;
    Servo sbkr, grabr, grabl,sbros;
    CRServo extl,extr;
    double currentAngle = 0;
    double exterR,exterL,extpowerR,extpowerL,extrlR,extrlL;
    double extr_pos, extl_pos;
    double extr_pos_was, extl_pos_was;
    double extr_raw_pos, extl_raw_pos;


    double Multiply = 0;
    double Multiply_defult = 0.3;
    double turnPower = 0;
    double forwardPower = 0;
    double sidePower = 0;

    double turnDrift = 0;
        double turnDrift_acel_seconds = 0.3;
        double turnDrift_decel_seconds = 0.1;
    double forwardDrift = 0;
        double forwardDrift_acel_seconds = 0.3;
        double forwardDrift_decel_seconds = 0.1;
    double sideDrift = 0;
        double sideDrift_acel_seconds = 0.3;
        double sideDrift_decel_seconds = 0.1;

    double min_drive_power = 0.2;
    double driftRate = 30;

    double cucle = 0;
    double cucle_max = 10;
    double mean_R_pos = 0;
    double mean_L_pos = 0;
    double mean_summ_R_pos = 0;
    double mean_summ_L_pos = 0;


    double forwardDrift_cel_coof = 1;
    double sideDrift_cel_coof = 1;
    double turnDrift_cel_coof = 1;
    double targAngle = 0;

    ElapsedTime driftCore = new ElapsedTime();

    boolean ignore_axel = false;
    double turnErr = 0, turnErrL = 0;
    double deltaHed = 0, deltaHedL = 0;

    boolean is_homed_ever = false;


    ElapsedTime sbkr_need_reset = new ElapsedTime();

    ElapsedTime continious_timer = new ElapsedTime();
    ;
    double pos = 0;



    ElapsedTime extention_time = new ElapsedTime();
    ElapsedTime color_pulse_timer = new ElapsedTime();
    ElapsedTime drive = new ElapsedTime();


    BNO055IMU Gyro;
    Orientation Orientation = new Orientation();
    DigitalChannel ch0, ch1;

    Telemetry dash = FtcDashboard.getInstance().getTelemetry();

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
        ch0 = hardwareMap.digitalChannel.get("0");
        ch1 = hardwareMap.digitalChannel.get("1");
        ch0.setMode(DigitalChannel.Mode.INPUT);
        ch1.setMode(DigitalChannel.Mode.INPUT);

        sbkr = hardwareMap.servo.get("sbkr");
        sbros = hardwareMap.servo.get("sbros");
        grabl = hardwareMap.servo.get("grabl");
        grabr = hardwareMap.servo.get("grabr");
//        s1 = hardwareMap.servo.get("servo2");
//        s2 = hardwareMap.servo.get("servo3");
        extl = hardwareMap.crservo.get("extl");
        extr = hardwareMap.crservo.get("extr");



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        Gyro = hardwareMap.get(BNO055IMU.class, "imu");


        Gyro.initialize(parameters);
//        OpenCvCamera webcam;
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
////        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//                FtcDashboard.getInstance().startCameraStream(webcam, 24);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//            }
//        });f
        String[] passord = new String[4];
        passord[0]="a";

        waitForStart();
        Exten.start();





        color_pulse_timer.reset();
        extention_time.reset();
        continious_timer.reset();
        sbkr_need_reset.reset();
        targAngle = Angle();
        driftCore.reset();
        while (opModeIsActive()) {
            // varing

            while (!gamepad1.start){
                if (gamepad1.dpad_up){
                    cucle_max += 1;
                    while (gamepad1.dpad_up){

                    }
                }
                else if (gamepad1.dpad_down){
                    cucle_max -= 1;
                    while (gamepad1.dpad_down){

                    }
                }
                telemetry.addData("Cycles:",cucle_max);
                telemetry.update();
            }
            while ((!gamepad1.back) && cucle<=cucle_max){
                cucle += 1;

                pos = -10000;
                waitExt();
                FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                pos = 6000;
                waitExt();
                if(cucle == 1){
                    extr_max = extr_raw_pos;
                    extl_max = extl_raw_pos;
                    mean_summ_R_pos = extr_raw_pos;
                    mean_summ_L_pos = extl_raw_pos;
                }
                else{
                    mean_summ_R_pos += extr_raw_pos;
                    mean_summ_L_pos += extl_raw_pos;

                    extr_max = mean_summ_R_pos/cucle;
                    extl_max = mean_summ_L_pos/cucle;
                }
                telemetry.addData("Calibration in progress...","");
                telemetry.addData("Current cycle:",cucle);
//                telemetry.addData("Extention R pos:",extr_raw_pos);
//                telemetry.addData("Extention L pos:",extl_raw_pos);
                telemetry.addData("Extention R max:",extr_max);
                telemetry.addData("Extention L max:",extl_max);

                telemetry.addData("To finish hold 'Back' on gamepad(A)","");
                telemetry.update();
                sleep(1000);
            }
            pos = 3000;
            telemetry.addData("Calibration Finished !","");
//                telemetry.addData("Extention R pos:",extr_raw_pos);
//                telemetry.addData("Extention L pos:",extl_raw_pos);
            telemetry.addData("Extention R max:",extr_max);
            telemetry.addData("Extention L max:",extl_max);

            telemetry.addData("Don`t turn off programm!","");
            telemetry.update();

            while (true){
            }










        }

    }
    public void waitExt(){
        extr_pos_was = FL.getCurrentPosition();
        extl_pos_was = FR.getCurrentPosition();
        sleep(3000);
        while (!(Math.abs(FL.getCurrentPosition()-extr_pos_was)<50)&&(Math.abs(FR.getCurrentPosition()-extl_pos_was))<50){

            extr_pos_was = extr_pos;
            extl_pos_was = extl_pos;
            sleep(3000);
        }
    }
    public double Angle() {
        Orientation = Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Orientation.firstAngle;
    }

    public void Move(){
        if (gamepad1.left_stick_x==0&&gamepad1.left_stick_y==0&&gamepad1.right_stick_x==0){
            mult = 1;
            drive.reset();
        }
        else{
            mult = Math.min(drive.seconds()*1,1);
        }

        if (driftCore.milliseconds()>=driftRate) {
            if (forwardDrift<gamepad1.left_stick_y) {
                if(forwardDrift>=0){
                    forwardDrift_cel_coof = forwardDrift_acel_seconds;
                }
                else{
                    forwardDrift_cel_coof = forwardDrift_decel_seconds;
                }
                if(ignore_axel){
                    forwardDrift_cel_coof = 0;
                }
                forwardDrift +=  Math.min(1/(1000/driftRate)/(Math.max(forwardDrift_cel_coof,0.0001)),(gamepad1.left_stick_y-forwardDrift));
            }
            else if(forwardDrift>gamepad1.left_stick_y){
                if(forwardDrift<=0){
                    forwardDrift_cel_coof = forwardDrift_acel_seconds;
                }
                else{
                    forwardDrift_cel_coof = forwardDrift_decel_seconds;
                }
                if(ignore_axel){
                    forwardDrift_cel_coof = 0;
                }
                forwardDrift -=  Math.min(1/(1000/driftRate)/(Math.max(forwardDrift_cel_coof,0.0001)),(forwardDrift-gamepad1.left_stick_y));

            }




            if (sideDrift<gamepad1.left_stick_x) {
                if(sideDrift>=0){
                    sideDrift_cel_coof = sideDrift_acel_seconds;
                }
                else{
                    sideDrift_cel_coof = sideDrift_decel_seconds;
                }
                if(ignore_axel){
                    sideDrift_cel_coof = 0;
                }
                sideDrift +=  Math.min(1/(1000/driftRate)/(Math.max(sideDrift_cel_coof,0.0001)),(gamepad1.left_stick_x-sideDrift));
            }
            else if(sideDrift>gamepad1.left_stick_x){
                if(sideDrift<=0){
                    sideDrift_cel_coof = sideDrift_acel_seconds;
                }
                else{
                    sideDrift_cel_coof = sideDrift_decel_seconds;
                }
                if(ignore_axel){
                    sideDrift_cel_coof = 0;
                }
                sideDrift -=  Math.min(1/(1000/driftRate)/(Math.max(sideDrift_cel_coof,0.0001)),(sideDrift-gamepad1.left_stick_x));
            }


            if (turnDrift<gamepad1.right_stick_x) {

                if(turnDrift>=0){
                    turnDrift_cel_coof = turnDrift_acel_seconds;
                }
                else{
                    turnDrift_cel_coof = turnDrift_decel_seconds;
                }
                if(ignore_axel){
                    turnDrift_cel_coof = 0;
                }
                turnDrift +=  Math.min(1/(1000/driftRate)/(Math.max(turnDrift_cel_coof,0.0001)),(gamepad1.right_stick_x-turnDrift));

            }
            else if(turnDrift>gamepad1.right_stick_x){
                dash.addData("positionx",1);
                if(turnDrift<=0){
                    turnDrift_cel_coof = turnDrift_acel_seconds;
                }
                else{
                    turnDrift_cel_coof = turnDrift_decel_seconds;
                }
                if(ignore_axel){
                    turnDrift_cel_coof = 0;
                }
                turnDrift -=  Math.min(1/(1000/driftRate)/(Math.max(turnDrift_cel_coof,0.0001)),(turnDrift-gamepad1.right_stick_x));
            }

            driftCore.reset();
        }

        forwardPower = SmartMax_zerocorr(forwardDrift,min_drive_power);
        sidePower = SmartMax_zerocorr(sideDrift,min_drive_power);

        double power = Math.sqrt((sidePower*sidePower)+(forwardPower*forwardPower));
        double radian = Math.atan2(forwardPower,sidePower);
        double Angle = Angle();

        deltaHed = Angle-deltaHedL;

        if(deltaHed>180){
            deltaHed -= 360;
        }
        else if(deltaHed<-180){
            deltaHed += 360;
        }
        currentAngle += deltaHed;

        if(turnDrift == 0){
            turnErr = targAngle-currentAngle;
            turnPower = (turnErr * pid_setting.turnKp + (turnErr - turnErrL) * pid_setting.turnKd)*0.4;
            turnErrL= turnErr;
        }
        else{
            targAngle = currentAngle;

            turnPower = -turnDrift*0.6;
        }
        FR.setPower(((power*Math.cos(radian-Math.PI/4+Math.toRadians(currentAngle)+Math.PI)*Math.sqrt(2))+turnPower)*(-Multiply));
        FL.setPower(((power*Math.cos(radian-3*Math.PI/4+Math.toRadians(currentAngle)+Math.PI)*Math.sqrt(2))-turnPower)*(Multiply));
        BR.setPower(((power*Math.cos(radian-3*Math.PI/4+Math.toRadians(currentAngle)+Math.PI)*Math.sqrt(2))+turnPower)*(-Multiply));
        BL.setPower(((power*Math.cos(radian-Math.PI/4+Math.toRadians(currentAngle)+Math.PI)*Math.sqrt(2))-turnPower)*(Multiply));
        dash.addData("positionx",BR.getCurrentPosition());
        dash.addData("positiony",BL.getCurrentPosition());
        dash.addData("ignorex",currentAngle);


//            BR.setPower((gamepad1.left_stick_x-gamepad1.left_stick_y+turnPower)*Multiply);
//            BL.setPower((-gamepad1.left_stick_x-gamepad1.left_stick_y-turnPower)*Multiply);
//            FR.setPower((-gamepad1.left_stick_x-gamepad1.left_stick_y+turnPower)*Multiply);
//            FL.setPower((gamepad1.left_stick_x-gamepad1.left_stick_y-turnPower)*Multiply);
        deltaHedL = Angle;









//        FR.setPower(((power*Math.cos(radian-Math.PI/4+Math.toRadians(currentAngle)+Math.PI)*Math.sqrt(2))+turnPower)*(-Multiply*mult));
//        FL.setPower(((power*Math.cos(radian-3*Math.PI/4+Math.toRadians(currentAngle)+Math.PI)*Math.sqrt(2))-turnPower)*(Multiply*mult));
//        BR.setPower(((power*Math.cos(radian-3*Math.PI/4+Math.toRadians(currentAngle)+Math.PI)*Math.sqrt(2))+turnPower)*(-Multiply*mult));
//        BL.setPower(((power*Math.cos(radian-Math.PI/4+Math.toRadians(currentAngle)+Math.PI)*Math.sqrt(2))-turnPower)*(Multiply*mult));







    }
    public double SmartMin(double a,double b){

        if(Math.abs(a)<=Math.abs(b)){
            return a;
        }
        else{
            if(a>=0) {
                return b;
            }
            else{
                return -b;
            }
        }
    }
    public double SmartMax_zerocorr(double a,double b){
        if(Math.abs(a-0)<0.07){
            return 0;
        }
        if(Math.abs(a)>=Math.abs(b)){
            return a;
        }
        else{
            if(a>=0) {
                return b;
            }
            else{
                return -b;
            }
        }
    }
    public double Normolaze_Enc(double M_pos, double M_min, double M_max, double range){
        // max pos must be positive and bigger min pos
        return (M_pos/(M_max-M_min))*range;
    }
    class Extention extends Thread{
        public void run(){
            while (opModeIsActive()){
                extr_pos =  -Normolaze_Enc(FL.getCurrentPosition(),extr_zero,extr_max,ext_range);
                extl_pos =  Normolaze_Enc(FR.getCurrentPosition(),extl_zero,extl_max,ext_range);
                extr_raw_pos = FL.getCurrentPosition();
                extl_raw_pos = FR.getCurrentPosition();


                exterL = -(pos + extl_pos); //left nigativ
                extpowerL = exterL * pid_setting.extpL + (exterL - extrlL) * pid_setting.extdL;
                extrlL = exterL;

//        exterR = pos - extr_pos;
                exterR = -(pos + extr_pos);//right positive

                extpowerR = exterR * pid_setting.extpR + (exterR - extrlR) * pid_setting.extdR;
                extrlR = exterR;

                extpowerR = SmartMin(extpowerR, 1);
                extpowerL = SmartMin(extpowerL, 1);



                if (ch0.getState() || extpowerR <= 0) {
                    if (extr_pos <= 5100 || extpowerR >= 0) {
                        extr.setPower(extpowerR);
                    } else {
                        extr.setPower(0);
//                        extpowerR = 0;
                    }
                } else {
                    extr.setPower(0);
//                    extpowerR = 0;
                }
                if (ch1.getState() || extpowerL <= 0) {
                    if (extl_pos >= -5100 || extpowerL >= 0) {
                        extl.setPower(extpowerL);
                    } else {
                        extl.setPower(0);
                        extpowerL = 0;
                    }
                } else {
                    extl.setPower(0);
                    extpowerL = 0;
                }
                if (pos > 5150) {
                    pos = 5150;
                }

                if (pos < -7000 && ch0.getState()) {
                    extpowerR = 1;
                    extr.setPower(1);
                }
                if (pos < -7000 && ch1.getState()) {
                    extpowerL = 1;
                    extl.setPower(1);
                }

                if ((!ch0.getState()) && (!ch1.getState())) {
                    FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    if (pos <= 0) {
                        pos = 0;
//                        gamepad2.rumble(500);
                    }
                    is_homed_ever = true;

                }

            }
        }
    }
}





