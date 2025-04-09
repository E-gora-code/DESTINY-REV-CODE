package org.firstinspires.ftc.teamcode.teamcode.control.actual;


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

//testyara
@TeleOp
public class Control_Alt_binds extends LinearOpMode {
    Init_Utilites initUtilites = new Init_Utilites();
    Drive_Base drive_base = new Drive_Base();
    Telemetry_manage telemetry_manage = new Telemetry_manage();
    Claw_controll claw_controll = new Claw_controll();
    PID_setting pid_setting = new PID_setting();
    // extention
    boolean simple_ext = true;

    // end
    // Gamepad drive binds

    double turn_stick_axis = 0;
    double forward_stick_axis = 0;
    double side_stick_axis = 0;
    double speed_controll_axis = 0;
    //end
    // Reset
    ElapsedTime presed_reset_ang_timer = new ElapsedTime();
    boolean is_reseted_ever = false;
    //end






    double extr_zero = 0, extl_zero = 0;
    double extr_max = 5672, extl_max = 3639;
    double ext_range = 5000;
    DcMotor FL, BL, FR, BR;
    Servo claw, grabr, grabl,sbros;
    CRServo extl,extr;
    double currentAngle = 0;
    double exterR,exterL,extpowerR,extpowerL,extrlR,extrlL;
    double extr_pos, extl_pos;
    double extr_raw_pos, extl_raw_pos;


    double Multiply = 0;
    double Multiply_defult = 0.6;
    double Multiply_turn = 1;
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

    double targAngle = 0;

    ElapsedTime driftCore = new ElapsedTime();

    boolean ignore_axel = false;
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
    boolean claw_toggle = false;
    boolean claw_press = false;
    double claw_poz = 0;
    double claw_grab_poz = 1;
    double claw_open_poz = 0.52;





    double sbros_pos_open = 0.8;
    // Открфтая позиция сброса яблок

    // закрытая в упор





    double claw_trigger_mult = 1;
    boolean claw_alt_mode_bind = false;

    ElapsedTime claw_need_reset = new ElapsedTime();
    ElapsedTime claw_last_alt = new ElapsedTime();
    ElapsedTime last_controlled_drive_base = new ElapsedTime();
    ElapsedTime hold_to_wake_homa = new ElapsedTime();
    ElapsedTime ext_timer = new ElapsedTime();

    ElapsedTime presed_reset_extencion_timer = new ElapsedTime();
    ElapsedTime continious_timer = new ElapsedTime();

    double x = 0;
    int click = 0;
    int clmult = 1;
    double pos = 0;


    ElapsedTime extention_time = new ElapsedTime();
    double extention_time_interval = 10;
    double extention_speed_multiply = 20;

    double color_pulse = 1;
    ElapsedTime color_pulse_timer = new ElapsedTime();
    ElapsedTime drive = new ElapsedTime();



    boolean can_homa_wake_up = true;

    boolean ishoma = false;
    ElapsedTime lastcl = new ElapsedTime();
    boolean iscl1 = false;
    boolean iscl = false;
    boolean hanging_state = false;
    BNO055IMU Gyro;
    Orientation Orientation = new Orientation();
    DigitalChannel ch0, ch1;

    Telemetry dash = FtcDashboard.getInstance().getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        initUtilites.start_init_instructions();
        waitForStart();

        drive_base.start();
        telemetry_manage.start();
        claw_controll.start();




        color_pulse_timer.reset();
        extention_time.reset();
        continious_timer.reset();
        claw_need_reset.reset();
        claw_last_alt.reset();
        targAngle = Angle();
        driftCore.reset();
        while (opModeIsActive()) {
            // varing
            extr_pos =  -Normolaze_Enc(FL.getCurrentPosition(),extr_zero,extr_max,ext_range);
            extl_pos =  Normolaze_Enc(FR.getCurrentPosition(),extl_zero,extl_max,ext_range);
            extr_raw_pos = FL.getCurrentPosition();
            extl_raw_pos = FR.getCurrentPosition();







            if(gamepad2.back){
                hanging_state = true;

            }
            if(hanging_state == false) {
                if (gamepad1.right_trigger > 0) {
                    sbros.setPosition((sbros_pos_open + gamepad1.right_trigger*(1-sbros_pos_open)));
                    continious_timer.reset();
                } else {
                    if (continious_timer.seconds() < 0.5) {
                        sbros.setPosition(sbros_pos_open);
                    } else {
                        sbros.setPosition(sbros_pos_open);
                    }
                }

            }
            else{
//                sbros.setPosition(0);
                if(simple_ext = true) {
                    extl.setPower(0.5);
                    extr.setPower(0.5);
                }
                if(gamepad2.start){
                    hanging_state = false;
                }
            }


            if(gamepad1.left_bumper||gamepad2.left_bumper){
                gamepad1.rumble(3000);
                gamepad2.rumble(3000);

            }
            if(color_pulse_timer.milliseconds()>1000) {
                color_pulse_timer.reset();
                color_pulse += 1;
                if(color_pulse>1){
                    color_pulse=0;
                    if(color_pulse==0){
                        gamepad2.setLedColor(1,0,0,100);
                        gamepad1.setLedColor(0,0,1,100);

                    }
                    else{
                        gamepad2.setLedColor(0.80,0.40,0,100);
                        gamepad1.setLedColor(0,0.50,1,100);
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



            if (gamepad1.a||gamepad2.right_bumper) {
                if(!claw_press){
                    claw_press = true;
                    claw_need_reset.reset();
                    if(!(gamepad1.start)) {
                        claw_toggle = !claw_toggle;
                    }
                }
                if(claw_toggle) {
                    claw_poz = claw_grab_poz;
                }else {
                    claw_poz = claw_open_poz;
                }
            }else {
                claw_press = false;
            }




            if (gamepad1.b) {
                if (!b_press) {
                    grab_toggle = !grab_toggle;
                    claw_toggle = false;
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

//            if ((pos_last - pos)!=0){
//                sbkr_toggle = false;
//            }






            if(simple_ext == false) {




                if (is_homed_ever) {
                    if (gamepad1.y || gamepad2.y) {
                        pos = 5080;
                    } else if (gamepad2.x) {
                        pos = 4032;
                    }
                }
                if (gamepad1.x || gamepad2.a) {
                    if(presed_reset_extencion_timer.milliseconds()>300) {
                        pos = -10000;
                    }
                }
                else{
                    presed_reset_extencion_timer.reset();
                }


                if ((gamepad1.dpad_right || gamepad2.dpad_up) && (!ext_press || ext_timer.milliseconds() >= 1000)) {
                    ext_pos_calk = 100 + 1000 * Math.max(gamepad1.left_trigger, gamepad2.left_trigger);
                    ext_timer.reset();
                    pos += ext_pos_calk;
                    ext_press = true;
                } else if ((gamepad1.dpad_left || gamepad2.dpad_down) && (!ext_press || ext_timer.milliseconds() >= 1000)) {
                    ext_pos_calk = 100 + 1000 * Math.max(gamepad1.left_trigger, gamepad2.left_trigger);
                    ext_timer.reset();
                    pos -= ext_pos_calk;
                    ext_press = true;
                } else {
                    if (!(gamepad1.dpad_left || gamepad2.dpad_down) && !(gamepad1.dpad_right || gamepad2.dpad_up)) {
                        ext_press = false;
                    }
                }

                if ((gamepad2.left_stick_y+gamepad2.right_stick_y) != 0) {
                    if (extention_time.milliseconds() > extention_time_interval) {
                        extention_time.reset();
                        pos -= ((gamepad2.left_stick_y+gamepad2.right_stick_y) * extention_speed_multiply) * (1 + (gamepad2.left_trigger * 2));
                    }
                }


                lastext = gamepad1.dpad_right;

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
            else{
                if (gamepad1.dpad_down){
                    extr.setPower(0.9);
                    extl.setPower(0.9);
                    claw_toggle = false;
                }
                else if(gamepad1.dpad_up){

                    extr.setPower(-0.9);
                    extl.setPower(-0.9);

                    claw_toggle = false;

                }
                else{
                    extr.setPower(gamepad2.left_stick_y+gamepad2.right_stick_y);
                    extl.setPower(gamepad2.left_stick_y+gamepad2.right_stick_y);
                    if ((gamepad2.left_stick_y+gamepad2.right_stick_y)!=0){
                        claw_toggle = false;
                    }

                }
            }

//        else{
//            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            pos = 0;
//        }


            gamepad_summ = gamepad1.left_stick_x+gamepad1.left_stick_y+gamepad1.right_stick_x+gamepad1.left_stick_y;




            if(gamepad1.ps){
                presed_reset_ang =true;
                if((presed_reset_ang_timer.seconds() >1) || !is_reseted_ever) {
                    gamepad1.rumble(100);
                    currentAngle = 0;
                    targAngle = 0;
                    is_reseted_ever = true;
                }
            }
            else{
                presed_reset_ang_timer.reset();
                presed_reset_ang =false;
            }





        }

    }

    class Telemetry_manage extends Thread{
        public void run() {
            while (opModeIsActive()){

                if(ishoma) {
                    addToBothTelemetry("-------Homak-------", " ");
                    addToBothTelemetry("Clicks", click);
                    addToBothTelemetry("PerClick", clmult);
                    addToBothTelemetry("Upgrade", (clmult * clmult * clmult * clmult));
                    addToBothTelemetry("-------------------------", " ");
                }
                addToBothTelemetry("-----|Extention|-----"," ");
                addToBothTelemetry("Target position", pos);
                addToBothTelemetry("extR position", Normolaze_Enc(extr_raw_pos,extr_zero,extr_max,ext_range));
                addToBothTelemetry("extL position", Normolaze_Enc(extl_raw_pos,extl_zero,extl_max,ext_range));
                addToBothTelemetry("ch0 state", ch0.getState());
                addToBothTelemetry("ch1 state", ch1.getState());
                addToBothTelemetry("Homed ever", is_homed_ever);
                addToBothTelemetry("-----------------------------"," ");

                addToBothTelemetry("-----|Drive Base|-----"," ");
                addToBothTelemetry("DriftF",forwardDrift);
                addToBothTelemetry("DriftS",sideDrift);
                addToBothTelemetry("DriftT",turnDrift);
                addToBothTelemetry("BL Encoder",BL.getCurrentPosition());
                addToBothTelemetry("BR Encoder",BR.getCurrentPosition());
                addToBothTelemetry("Drift Calculation",(1/(1000/driftRate))/((3)));
                addToBothTelemetry("-----------------------------"," ");

                telemetry.addData("tog", claw_toggle);
                dash.addData("ext", FL.getCurrentPosition());
                dash.update();




                telemetry.addData("Zahvat", claw_poz);

                updateBothTelemrtry();
            }
        }
        public void addToBothTelemetry(String caption,Object value){
            telemetry.addData(caption,value);
            dash.addData(caption,value);
        }
        public void updateBothTelemrtry(){
            telemetry.update();
            dash.update();
        }

    }
    public void doHoma(){
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
    }
    class Claw_controll extends Thread{
        public void run() {
            claw_alt_mode_bind = gamepad1.right_bumper;
            if(claw_alt_mode_bind==true){
                claw_last_alt.reset();
            }


            if((last_controlled_drive_base.seconds() > 20)&&(claw_need_reset.seconds()>30)){
                claw_toggle = false;
            }
            if(claw_last_alt.seconds()>1) {
                if(gamepad1.right_trigger>0) {
                    claw_need_reset.reset();
                    if(claw_toggle ==false) {
                        claw_poz = (((gamepad1.right_trigger * claw_trigger_mult) * (1 - claw_open_poz)) + claw_open_poz) * claw_grab_poz;
                    }
                    else {
                        claw_poz = (((1-(gamepad1.right_trigger*gamepad1.right_trigger)) * (1 - claw_open_poz)) + claw_open_poz) * claw_grab_poz;
                    }
                }
                else{
                    set_claw_pos_state();
                }
            }
            else{
                set_claw_pos_state();
            }
            claw.setPosition(claw_poz);

        }
        public void set_claw_pos_state(){
            if(claw_toggle) {
                claw_poz = claw_grab_poz;
            }else {
                claw_poz = claw_open_poz;
            }
        }
    }
    class Drive_Base extends Thread {
        public void run() {
            while (opModeIsActive()) {
                // Gamepad binds

                turn_stick_axis = gamepad1.right_stick_x;
                forward_stick_axis = gamepad1.left_stick_y;
                side_stick_axis = gamepad1.left_stick_x;
                speed_controll_axis = gamepad1.left_trigger;

                // Set wariables
                if (turn_stick_axis != 0 || forward_stick_axis != 0 || side_stick_axis != 0) {
                    if(last_controlled_drive_base.seconds()>1) {
                        last_controlled_drive_base.reset();
                    }
                }
                else if(last_controlled_drive_base.seconds() > 10){
                    FR.setPower(0);
                    FL.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                }
                if (last_controlled_drive_base.seconds()>20){
                    is_reseted_ever = false;
                }

                if (speed_controll_axis > 0.3) {
                    ignore_axel = true;
                    Multiply = speed_controll_axis*1;
                } else {
                    Multiply = Multiply_defult;
                    ignore_axel = false;
                }


                if (last_controlled_drive_base.seconds() < 10) {


                    if (driftCore.milliseconds() >= driftRate) {
                        forwardDrift = Accel_block(forwardDrift,forward_stick_axis,forwardDrift_acel_seconds,forwardDrift_decel_seconds);
                        sideDrift = Accel_block(sideDrift,side_stick_axis,sideDrift_acel_seconds,sideDrift_decel_seconds);
                        turnDrift = Accel_block(turnDrift,turn_stick_axis,turnDrift_acel_seconds,turnDrift_decel_seconds);

                        driftCore.reset();
                    }


                    forwardPower = SmartMax_zerocorr(forwardDrift, min_drive_power);
                    sidePower = SmartMax_zerocorr(sideDrift, min_drive_power);

                    double power = Math.sqrt((sidePower * sidePower) + (forwardPower * forwardPower));
                    double radian = Math.atan2(forwardPower, sidePower);
                    double Angle = Angle();

                    deltaHed = Angle - deltaHedL;

                    if (deltaHed > 180) {
                        deltaHed -= 360;
                    } else if (deltaHed < -180) {
                        deltaHed += 360;
                    }
                    currentAngle += deltaHed;

                    if (turnDrift == 0) {
                        turnErr = targAngle - currentAngle;
                        turnPower = (turnErr * pid_setting.turnKp + (turnErr - turnErrL) * pid_setting.turnKd) *  0.4;
                        turnErrL = turnErr;
                    } else {
                        targAngle = currentAngle;

                        turnPower = -turnDrift * Multiply_turn;
                    }
                    double mult_on_press_L_stick = 1;
                    double mult_on_press_R_stick = 1;
                    if(gamepad1.left_stick_button){
                        mult_on_press_L_stick = 1/Multiply;
                    }
                    if(gamepad1.right_stick_button){
                        mult_on_press_R_stick = 1/Multiply;
                    }

                    FR.setPower(((power * Math.cos(radian - Math.PI / 4 + Math.toRadians(currentAngle) + Math.PI) * Math.sqrt(2))*mult_on_press_L_stick + turnPower*mult_on_press_R_stick) * (-Multiply)*0.7);
                    FL.setPower(((power * Math.cos(radian - 3 * Math.PI / 4 + Math.toRadians(currentAngle) + Math.PI) * Math.sqrt(2))*mult_on_press_L_stick - turnPower*mult_on_press_R_stick) * (Multiply));
                    BR.setPower(((power * Math.cos(radian - 3 * Math.PI / 4 + Math.toRadians(currentAngle) + Math.PI) * Math.sqrt(2))*mult_on_press_L_stick + turnPower*mult_on_press_R_stick) * (-Multiply));
                    BL.setPower(((power * Math.cos(radian - Math.PI / 4 + Math.toRadians(currentAngle) + Math.PI) * Math.sqrt(2))*mult_on_press_L_stick - turnPower*mult_on_press_R_stick) * (Multiply));

                    deltaHedL = Angle;


                }
            }
        }
        public double Accel_block(double whatDrift,double what_stick_axis,double whatDrift_acel_seconds,double whatDrift_decel_seconds){
            double whatDrift_cel_coof = 0;
            if (whatDrift < what_stick_axis) {
                if (whatDrift >= 0) {
                    whatDrift_cel_coof = whatDrift_acel_seconds;
                } else {
                    whatDrift_cel_coof = whatDrift_decel_seconds;
                }
                if (ignore_axel) {
                    whatDrift_cel_coof = 0;
                }
                whatDrift += Math.min(1 / (1000 / driftRate) / (Math.max(whatDrift_cel_coof, 0.0001)), (what_stick_axis - whatDrift));
            } else if (whatDrift > what_stick_axis) {
                if (whatDrift <= 0) {
                    whatDrift_cel_coof = whatDrift_acel_seconds;
                } else {
                    whatDrift_cel_coof = whatDrift_decel_seconds;
                }
                if (ignore_axel) {
                    whatDrift_cel_coof = 0;
                }
                whatDrift -= Math.min(1 / (1000 / driftRate) / (Math.max(whatDrift_cel_coof, 0.0001)), (whatDrift - what_stick_axis));

            }
            return whatDrift;
        }
    }
    public class Init_Utilites{
        public void start_init_instructions(){
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


            claw = hardwareMap.servo.get("sbkr");
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

        }
    }
    public double Angle() {
        Orientation = Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Orientation.firstAngle;
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
}





