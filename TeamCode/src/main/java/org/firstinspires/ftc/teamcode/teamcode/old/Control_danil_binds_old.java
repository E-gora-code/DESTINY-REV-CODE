package org.firstinspires.ftc.teamcode.teamcode.old;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
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
import org.firstinspires.ftc.teamcode.teamcode.PID_setting;


//@TeleOp
public class Control_danil_binds_old extends LinearOpMode {
                                                                                                                                                                                                                                                                                            boolean xkfi = false;
    PID_setting pid_setting = new PID_setting();
    double extr_zero = 0, extl_zero = 0;
    double extr_max = 5170, extl_max;
    DcMotor FL, BL, FR, BR;
    Servo sbkr, grabr, grabl,sbros;
    CRServo extl,extr;
    double currentAngle = 0;
    double exterR,exterL,extpowerR,extpowerL,extrlR,extrlL;
    double extr_pos, extl_pos;
    double Multiply = 0.2;
    double turnPower = 0;
    double targAngle = 0;
    double turnErr = 0, turnErrL = 0;
    double deltaHed = 0, deltaHedL = 0;
    double ext_pos_calk = 0;
    double gamepad_summ = 0;
    boolean lastext = false;
    boolean presed_reset_ang = false;
    boolean ext_press=false;
    boolean is_homed_ever = false;
    boolean b_press = false;
    boolean grab_toggle = false;
    boolean sbkr_toggle = false;
    boolean sbkr_press = false;
    ElapsedTime homatimr = new ElapsedTime();
    ElapsedTime lastctrl = new ElapsedTime();
    ElapsedTime hold_to_wake_homa = new ElapsedTime();
    ElapsedTime ext_timer = new ElapsedTime();
    ElapsedTime presed_reset_ang_timer = new ElapsedTime();
    ElapsedTime presed_reset_extencion_timer = new ElapsedTime();
    double x = 0;
    int click = 0;
    int clmult = 1;
    double pos = 0;
    double gm2ls_summ = 0;
    ElapsedTime drive = new ElapsedTime();


    ElapsedTime extention_time = new ElapsedTime();
    double extention_time_interval = 10;

    double color_pulse = 1;
    ElapsedTime color_pulse_timer = new ElapsedTime();
    ElapsedTime continious_timer = new ElapsedTime();

    boolean can_homa_wake_up = true;

    boolean ishoma = false;
    ElapsedTime lastcl = new ElapsedTime();
    boolean iscl1 = false;
    boolean iscl = false;
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





        continious_timer.reset();
        color_pulse_timer.reset();
        extention_time.reset();
        targAngle = Angle();
        while (opModeIsActive()) {
            // varing
            extr_pos =  FL.getCurrentPosition();
            extl_pos =  FR.getCurrentPosition();




            if(click<0){
                click=0;
            }
            if (gamepad1.start) {
                if (lastcl.milliseconds()>500||!iscl) {
                    click += 1 * clmult;
                    lastcl.reset();
                    iscl = true;
                }
                if(can_homa_wake_up&&hold_to_wake_homa.seconds()>=4) {
                    ishoma = true;
                    hold_to_wake_homa.reset();
                }
            } else {
                lastcl.reset();
                iscl = false;
                hold_to_wake_homa.reset();
            }
            if (gamepad1.back) {
                if (!iscl1 && click >= (clmult * clmult * clmult * clmult)) {
                    click -= clmult * clmult * clmult * clmult;
                    clmult += 1;
                    iscl1 = true;
                }
            } else {
                iscl1 = false;
            }
            if(ishoma) {
                telemetry.addData("-------Homak-------", " ");
                dash.addData("-------Homak-------", " ");
                telemetry.addData("Clicks", click);
                dash.addData("Clicks", click);
                telemetry.addData("PerClick", clmult);
                dash.addData("PerClick", clmult);
                telemetry.addData("Upgrade", (clmult * clmult * clmult * clmult));
                dash.addData("Upgrade", (clmult * clmult * clmult * clmult));
                telemetry.addData("-------------------------", " ");
                dash.addData("-------------------------", " ");
            }

            telemetry.addData("ch0", ch0.getState());
            telemetry.addData("ch1", ch1.getState());
//            telemetry.addData("exRpow", extpowerR);
            telemetry.addData("homed", is_homed_ever);
            telemetry.addData("tog", grab_toggle);

            telemetry.addData("extR", extr_pos);
            telemetry.addData("extL", extl_pos);
            telemetry.addData("pos", pos);
            telemetry.addData("extL_p",extpowerL);
            telemetry.addData("extR_p",extpowerR);
            telemetry.addData("BL",BL.getCurrentPosition());
            telemetry.addData("BR",BR.getCurrentPosition());

//            dash.addData("angle",currentAngle);
//            dash.addData("angletarg",targAngle);
//            dash.addData("Angle", Angle());
            telemetry.update();
            dash.update();


            if(gamepad2.right_trigger>0) {
                sbros.setPosition((1-gamepad2.right_trigger));
                continious_timer.reset();
            }
            else{
                if(continious_timer.seconds()<0.5) {
                    sbros.setPosition(0.8);
                }
                else{
                    sbros.setPosition(0.95);
                }
            }


            if(gamepad2.left_bumper){
                gamepad1.rumble(3000);
                gamepad2.rumble(3000);

            }
            if(color_pulse_timer.milliseconds()>1000) {
                color_pulse_timer.reset();
                color_pulse += 1;
                if(color_pulse>1){
                    color_pulse=0;
                    if(color_pulse==0){
                        gamepad2.setLedColor(255,0,0,1000);
                        gamepad1.setLedColor(0,0,255,1000);

                    }
                    else if (color_pulse==1){
                        gamepad2.setLedColor(255,255,0,1000);
                        gamepad1.setLedColor(0,255,255,1000);
                    }
                }

            }






//            if(gamepad1.a||gamepad2.a){
//                s.setPosition(0.35);
//            }
//            else if(gamepad1.b||gamepad2.b){
//                s.setPosition(0.65);
//            }
//            else if(gamepad2.left_stick_y!=0){
//                s.setPosition(0.5-(gamepad2.left_stick_y*0.3));
//            }
//            else {
//                s.setPosition(0.5);
//            }
//            s1.setPosition(gamepad2.left_trigger);
//            s2.setPosition(gamepad2.right_trigger);
//            if(gamepad2.x){
//                s.setPosition(1);
//            }




            if (gamepad1.right_bumper||gamepad2.right_bumper) {
                if(!sbkr_press){
                    sbkr_press = true;
                    sbkr_toggle = !sbkr_toggle;
                }
                if(sbkr_toggle) {
                    sbkr.setPosition(0.8);
                }else {
                    sbkr.setPosition(0.3);
                }
            }else {
                sbkr_press = false;
            }








            if (gamepad1.right_trigger>0) {
                if (!b_press) {
                    grab_toggle = !grab_toggle;
                }
                b_press = true;
            } else {
                b_press = false;
            }
            if (grab_toggle) {
                grabr.setPosition(0.05);
                grabl.setPosition(0.75);
            } else {
                grabr.setPosition(0.5);
                grabl.setPosition(0.2);
            }

//            if(gamepad2.left_bumper||gamepad1.left_bumper){
//                while (true){
//                    extl.setPower(0);
//                    extr.setPower(0);
//                }
//            }

            if(true) {




                if (is_homed_ever) {
                    if (gamepad2.y) {
                        pos = 5080;
                    } else if (gamepad2.x) {
                        pos = 4032;
                    }
                }
                if (gamepad2.a) {
                    if(presed_reset_extencion_timer.milliseconds()>300) {
                        pos = -10000;
                    }
                }
                else{
                    presed_reset_extencion_timer.reset();
                }


                if (( gamepad2.dpad_up) && (!ext_press || ext_timer.milliseconds() >= 1000)) {
                    ext_pos_calk = 100 + 1000 * (gamepad2.left_trigger);
                    ext_timer.reset();
                    if (ishoma) {
                        if (click - (ext_pos_calk * 0.05) > 70) {
                            pos += ext_pos_calk;
                            click -= ext_pos_calk * 0.05;
                        }
                    } else {
                        pos += ext_pos_calk;
                    }
                    ext_press = true;
                } else if (( gamepad2.dpad_down) && (!ext_press || ext_timer.milliseconds() >= 1000)) {
                    ext_pos_calk = 100 + 1000 * (gamepad2.left_trigger);
                    ext_timer.reset();
                    if (ishoma) {
                        if (click - (ext_pos_calk * 0.05) > 70) {
                            pos -= ext_pos_calk;
                            click -= ext_pos_calk * 0.05;
                        }
                    } else {
                        pos -= ext_pos_calk;
                    }
                    ext_press = true;
                } else {
                    if (!(gamepad2.dpad_down) && !( gamepad2.dpad_up)) {
                        ext_press = false;
                    }
                }

                if ((gamepad2.left_stick_y+gamepad2.right_stick_y) != 0) {
                    if (extention_time.milliseconds() > extention_time_interval) {
                        extention_time.reset();
                        pos -= ((gamepad2.left_stick_y+gamepad2.right_stick_y) * 5) * (1 + (gamepad2.left_trigger * 2));
                    }
                }


                lastext = gamepad1.dpad_right;

                exterL = -(pos + extl_pos); //left nigativ
                extpowerL = exterL * pid_setting.extpL + (exterL - extrlL) * pid_setting.extdL;
                extrlL = exterL;

//        exterR = pos - extr_pos;
                exterR = -(extl_pos + extr_pos);//right positive

                extpowerR = exterR * pid_setting.extpR + (exterR - extrlR) * pid_setting.extdR;
                extrlR = exterR;

                extpowerR = SmartMin(extpowerR, 1);
                extpowerL = SmartMin(extpowerL, 0.8);


                if (ch0.getState() || extpowerR >= 0) {
                    if (extr_pos <= 5100 || extpowerR <= 0) {
                        extr.setPower(-extpowerR);
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
            else{
                if (gamepad1.dpad_right){
                    extr.setPower(0.6);
                    extl.setPower(-0.43);
                }
                else if(gamepad1.dpad_left){

                    extr.setPower(-0.6);
                    extl.setPower(0.43);

                }
                else{
                    extr.setPower(0);
                    extl.setPower(0);

                }
            }

//        else{
//            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            pos = 0;
//        }


            gamepad_summ = gamepad1.left_stick_x+gamepad1.left_stick_y+gamepad1.right_stick_x+gamepad1.left_stick_y;

            if (gamepad1.left_trigger > 0.3) {
                                                                                                                                                                                                                                                                                                                                            xkfi=true;
                if (ishoma) {
                    if (click > 50) {
                        if (homatimr.milliseconds() > 100 && gamepad_summ!=0) {
                            click -= 5;
                            homatimr.reset();
                        }
                        Multiply = gamepad1.left_trigger;
                    } else if (click > 10) {
                        if (homatimr.milliseconds() > 100 && gamepad_summ!=0) {
                            click -= 1;
                            homatimr.reset();
                        }
                        Multiply = gamepad1.left_trigger * 0.5;
                    } else {
                        Multiply = 0.3;
                    }

                } else {
                    Multiply = gamepad1.left_trigger*1.6;
                }
            } else {
                                                                                                                                                                                                                                                                                                                                                                                                                        xkfi=true;
                Multiply = 0.6;
            }


//            BR.setPower((gamepad1.left_stick_x-gamepad1.left_stick_y+gamepad1.right_stick_x)*Multiply);
//            BL.setPower((-gamepad1.left_stick_x-gamepad1.left_stick_y-gamepad1.right_stick_x)*Multiply);
//            FR.setPower((-gamepad1.left_stick_x-gamepad1.left_stick_y+gamepad1.right_stick_x)*Multiply);
//            FL.setPower((gamepad1.left_stick_x-gamepad1.left_stick_y-gamepad1.right_stick_x)*Multiply);
//            s.setPosition(gamepad2.left_stick_y);
//            s1.setPosition(1-gamepad2.right_stick_y);

//            if(abs(turnErr)>180) {
//                turnErr = -turnErr;
//            }
//            telemetry.addData("Gyro",Angle());
//            telemetry.addData("Target",targAngle);
//            telemetry.addData("t1",turnErr1);
//            telemetry.addData("t2",turnErr2);
//            telemetry.update();
            if(gamepad1.dpad_up){
                presed_reset_ang =true;
                if(presed_reset_ang_timer.seconds() >2) {
                    gamepad1.rumble(100);
                    currentAngle = 0;
                    targAngle = 0;
                }
            }
            else{
                presed_reset_ang_timer.reset();
                presed_reset_ang =false;
            }

            if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) {
                if(lastctrl.seconds()>1) {
                    lastctrl.reset();
                }
            }
//            if (lastctrl.seconds() < 10) {
                Move();
//            }
//            else if(lastctrl.seconds() > 10){
//                FR.setPower(0);
//                FL.setPower(0);
//                BR.setPower(0);
//                BL.setPower(0);
//            }
//            BR.setPower((gamepad1.left_stick_x-gamepad1.left_stick_y+turnPower)*Multiply);
//            BL.setPower((-gamepad1.left_stick_x-gamepad1.left_stick_y-turnPower)*Multiply);
//            FR.setPower((-gamepad1.left_stick_x-gamepad1.left_stick_y+turnPower)*Multiply);
//            FL.setPower((gamepad1.left_stick_x-gamepad1.left_stick_y-turnPower)*Multiply);




                                                                                                                                                                                                                                                                                                                                if(xkfi==false){turnPower=0/0;}

        }

    }

    public double Angle() {
        if (gamepad1.left_stick_x==0){
            drive.reset();
            double mult = 1;
        }
        if (gamepad1.left_stick_y==0){
            drive.reset();
            double mult = 1;
        }
        if (gamepad1.right_stick_x==0){
            drive.reset();
            double mult = 1;
        }
        else{
            double mult = drive.milliseconds()*0.3;
        }
        Orientation = Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Orientation.firstAngle;
    }

    public void Move(){
        double power = Math.sqrt((gamepad1.left_stick_x*gamepad1.left_stick_x)+(gamepad1.left_stick_y*gamepad1.left_stick_y));
        double radian = Math.atan2(gamepad1.left_stick_y,gamepad1.left_stick_x);
        double Angle = Angle();

        deltaHed = Angle-deltaHedL;

        if(deltaHed>180){
            deltaHed -= 360;
        }
        else if(deltaHed<-180){
            deltaHed += 360;
        }
        currentAngle += deltaHed;

        if(gamepad1.right_stick_x == 0){
            turnErr = targAngle-currentAngle;
            turnPower = turnErr * pid_setting.turnKp*0.5 + (turnErr - turnErrL) * pid_setting.turnKd;
            turnErrL= turnErr;
        }
        else{
            targAngle = currentAngle;
            turnPower = -gamepad1.right_stick_x*0.8;
        }
        FR.setPower(((power*Math.cos(radian-Math.PI/4+Math.toRadians(currentAngle)+Math.PI)*Math.sqrt(2))+turnPower*1.3)*-Multiply);
        FL.setPower(((power*Math.cos(radian-3*Math.PI/4+Math.toRadians(currentAngle)+Math.PI)*Math.sqrt(2))-turnPower*1.3)*Multiply);
        BR.setPower(((power*Math.cos(radian-3*Math.PI/4+Math.toRadians(currentAngle)+Math.PI)*Math.sqrt(2))+turnPower*1.3)*-Multiply);
        BL.setPower(((power*Math.cos(radian-Math.PI/4+Math.toRadians(currentAngle)+Math.PI)*Math.sqrt(2))-turnPower*1.3)*Multiply);
        dash.addData("positionx",BR.getCurrentPosition());
        dash.addData("positiony",BL.getCurrentPosition());
        dash.addData("ignorex",currentAngle);

//            BR.setPower((gamepad1.left_stick_x-gamepad1.left_stick_y+turnPower)*Multiply);
//            BL.setPower((-gamepad1.left_stick_x-gamepad1.left_stick_y-turnPower)*Multiply);
//            FR.setPower((-gamepad1.left_stick_x-gamepad1.left_stick_y+turnPower)*Multiply);
//            FL.setPower((gamepad1.left_stick_x-gamepad1.left_stick_y-turnPower)*Multiply);
        deltaHedL = Angle;

















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
}





