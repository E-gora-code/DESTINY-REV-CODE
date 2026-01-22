package org.firstinspires.ftc.teamcode.teamcode.control.actual;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.generic_classes.OpRobotSystemsFramework;
import org.firstinspires.ftc.teamcode.teamcode.PID_setting;
import org.firstinspires.ftc.teamcode.teamcode.openCV.CameraOverlay;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.generic_classes.OpModeFramework;
@TeleOp
public class $Main_Control extends OpRobotSystemsFramework {
    OpenCvCamera webcam;
    private Acceleration acceleration;
    Init_Utilites initUtilites = new Init_Utilites();

    GamepadBinds gamepadBinds = new GamepadBinds();
    Drive_system drive_system = new Drive_system();
    Telemetry_manage telemetry_manage = new Telemetry_manage();
    PID_setting pid_setting = new PID_setting();
    // extention
    boolean simple_ext = true;

    // end
    // Gamepad drive binds

        double turn_stick_axis = 0;
        double forward_stick_axis = 0;
        double side_stick_axis = 0;
        double speed_controll_axis = 0;
        boolean pos_reset_bind = false;

        boolean angle_snap_bind = false;

        boolean drive_base_accel_move_bind = false;
        boolean drive_base_accel_turn_bind = false;

    //Spindexter binds
        boolean spindexter_intake_bind = false;
        boolean spindexter_shoot_bind = false;

    //Silly binds
        boolean dual_rumble_bind = false;

    //end
    // Reset
        ElapsedTime presed_reset_ang_timer = new ElapsedTime();
        boolean is_reseted_ever = false;
    //end




    double currentAngle = 0;

    double Multiply = 0;

    double Multiply_turn = 0.8;
    double Multiply_defult = 0.6;
    double turnPower = 0;
    double forwardPower = 0;
    double sidePower = 0;

    boolean is_snap_turned_recent = false;
    double snap_180_mode = 0;



    double min_drive_power = 0.2;
    double driftRate = 30;

    double targAngle = 0;

    ElapsedTime driftCore = new ElapsedTime();

    boolean ignore_axel = false;
    double turnErr = 0, turnErrL = 0;
    double deltaHed = 0, deltaHedL = 0;
    double gamepad_summ = 0;
    boolean presed_reset_ang = false;

    boolean claw_toggle = false;
    double claw_poz = 0;



    double sbros_pos_open = 0.6;

    ElapsedTime claw_need_reset = new ElapsedTime();
    ElapsedTime claw_last_alt = new ElapsedTime();
    ElapsedTime last_controlled_drive_base = new ElapsedTime();
    ElapsedTime hold_to_wake_homa = new ElapsedTime();
    ElapsedTime ext_timer = new ElapsedTime();

    ElapsedTime presed_reset_extencion_timer = new ElapsedTime();
    ElapsedTime continious_timer = new ElapsedTime();

    ElapsedTime snap_angle_timer = new ElapsedTime();

    ElapsedTime simple_ext_top_shortcut_timer = new ElapsedTime();
    ElapsedTime simple_ext_homing_timer = new ElapsedTime();

    double x = 0;
    int click = 0;
    int clmult = 1;
    double pos = 0;


    ElapsedTime extention_time = new ElapsedTime();
    double extention_time_interval = 10;
    double extention_speed_multiply = 20;

    double ext_button_miltyply = 150;

    double ext_button_sleep_time_ms = 100;

    double ext_mult_speed_up = 100;

    double color_pulse = 1;
    ElapsedTime color_pulse_timer = new ElapsedTime();



    boolean can_homa_wake_up = true;

    boolean ishoma = false;
    ElapsedTime lastcl = new ElapsedTime();
    boolean iscl1 = false;
    boolean iscl = false;
    boolean hanging_state = false;
    Orientation Orientation = new Orientation();



    @Override
    public void runOpMode() throws InterruptedException {
        initUtilites.start_init_instructions();
        waitForStart();

        gamepadBinds.start();
        drive_system.start();
        telemetry_manage.start();




        color_pulse_timer.reset();
        extention_time.reset();
        continious_timer.reset();
        claw_need_reset.reset();
        claw_last_alt.reset();
        targAngle = Angle();
        driftCore.reset();
        snap_angle_timer.reset();
        simple_ext_top_shortcut_timer.reset();
        simple_ext_homing_timer.reset();
        while (opModeIsActive()) {

            motors.class_tick();





            if(dual_rumble_bind){
                gamepad1.rumble(3000);
                gamepad2.rumble(3000);

            }







            if(pos_reset_bind){
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
    public class GamepadBinds extends Thread{
        public void run() {
            while (opModeIsActive()) {
                default_profile();
            }
        }
        public void default_profile(){
            turn_stick_axis = gamepad1.right_stick_x;
            forward_stick_axis = gamepad1.left_stick_y;
            side_stick_axis = gamepad1.left_stick_x;
            speed_controll_axis = gamepad1.left_trigger;
            drive_base_accel_move_bind = gamepad1.left_stick_button;
            drive_base_accel_turn_bind = gamepad1.right_stick_button;

            angle_snap_bind = is_greater(Math.abs(gamepad1.right_stick_y),0.9);
            if(angle_snap_bind) {
                snap_180_mode = isAwayFronZero(gamepad1.right_stick_y, 0.5);
            }

            pos_reset_bind = gamepad1.dpad_left||gamepad1.ps;

            gamepad_summ = gamepad1.left_stick_x+gamepad1.left_stick_y+gamepad1.right_stick_x+gamepad1.left_stick_y;

            dual_rumble_bind = gamepad1.left_bumper||gamepad2.left_bumper;

        }

        public double statement_double(double value, boolean condition){
            if(condition) {
                return value;
            }
            else {
                return 0;
            }
        }
        public boolean is_greater(double value, double max){
            if(value<=max) {
                return false;
            }
            else {
                return true;
            }
        }
        public double isAwayFronZero(double a,double dist){
            if(Math.abs(a)>=dist){
                return a;
            }
            else{return 0;}
        }
    }
    public class Telemetry_manage extends Thread{
        public void run() {
            while (opModeIsActive()){
                doHoma();

                if(ishoma) {
                    addToBothTelemetry("-------Homak-------", " ");
                    addToBothTelemetry("Clicks", click);
                    addToBothTelemetry("PerClick", clmult);
                    addToBothTelemetry("Upgrade", (clmult * clmult * clmult * clmult));
                    addToBothTelemetry("-------------------------", " ");
                }
                if (acceleration != null) {
                    addToBothTelemetry("Accel X", acceleration.xAccel);
                    addToBothTelemetry("Accel Y", acceleration.yAccel);
                    addToBothTelemetry("Accel Z", acceleration.zAccel);
                } else {
                    addToBothTelemetry("Accel", "No data!");
                }
                addToBothTelemetry("-----|Drive Base|-----"," ");
                addToBothTelemetry("DriftF",drive_system.forwardDrift);
                addToBothTelemetry("DriftS",drive_system.sideDrift);
                addToBothTelemetry("DriftT",drive_system.turnDrift);
                addToBothTelemetry("BL Encoder",BL.getCurrentPosition());
                addToBothTelemetry("BR Encoder",BR.getCurrentPosition());
                addToBothTelemetry("Drift Calculation",(1/(1000/driftRate))/((3)));
                addToBothTelemetry("-----------------------------"," ");
                addToBothTelemetry("-----|Rotation|-----"," ");
                addToBothTelemetry("Current Angle", currentAngle);
                addToBothTelemetry("Target Angle", targAngle);
                addToBothTelemetry("-----------------------------"," ");
                addToBothTelemetry("-----|Binds|-----"," ");
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
//            dash.update();
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

    }


    public class Drive_system extends Thread {
        double turnDrift = 0;
        double turnDrift_acel_seconds = 0.3;
        double turnDrift_decel_seconds = 0.1;
        double forwardDrift = 0;
        double forwardDrift_acel_seconds = 0.3;
        double forwardDrift_decel_seconds = 0.1;
        double sideDrift = 0;
        double sideDrift_acel_seconds = 0.3;
        double sideDrift_decel_seconds = 0.1;
        public Drive_system(){

        }
        public void run() {
            while (opModeIsActive()) {
                acceleration = gyro.Axel();
                driveBase.class_tick();


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

                    if(angle_snap_bind){
                        snap_angle_timer.reset();
                        targAngle = Math.round(targAngle/90)*90;
                    }

                    if ((turnDrift == 0)||(snap_angle_timer.seconds()<1)) {
                        turnErr = targAngle - currentAngle;
                        turnPower = (turnErr * pid_setting.turnKp + (turnErr - turnErrL) * pid_setting.turnKd) * 0.4;
                        turnErrL = turnErr;
                    }
                    else {
                        targAngle = currentAngle;

                        turnPower = -turnDrift * Multiply_turn;
                    }
                    double seconds_to_snap_turn = 1.5;
                    if ((turnDrift != 0)&&(snap_angle_timer.seconds()<seconds_to_snap_turn)) {
                        if(!is_snap_turned_recent){
                            if(Math.abs(turnDrift)>0.5) {
                                if (snap_180_mode < 0) {
                                    targAngle -= 90 * getZnak(turnDrift);
                                }
                                else {
                                    targAngle -= 180 * getZnak(turnDrift);
                                }

                                is_snap_turned_recent = true;
                            }
                        }

                    }
                    else if(snap_angle_timer.seconds()>=seconds_to_snap_turn){
                        is_snap_turned_recent = false;
                    }
                    if(snap_angle_timer.seconds()>seconds_to_snap_turn){
                        is_snap_turned_recent = false;
                    }


                    double mult_on_press_L_stick = 1;
                    double mult_on_press_R_stick = 1;
                    if(drive_base_accel_move_bind){
                        mult_on_press_L_stick = 1/Multiply;
                    }
                    if(drive_base_accel_turn_bind){
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

            selfInit();
            initAllSystems();


//            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//    //        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            CameraOverlay detector = new CameraOverlay();
//            webcam.setPipeline(detector);
//            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//                @Override
//                public void onOpened() {
//                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
//
//                    FtcDashboard.getInstance().startCameraStream(webcam, 16);
//                }
//
//                @Override
//                public void onError(int errorCode) {
//                }
//            });
        }
    }
    public double Angle() {
        Orientation = gyro.Angle();
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
    public void setExtRightPower(double power){
        power = -power;
        if(ch0.getState()){
            extr.setPower(power);
        }else if(power>0){
            extr.setPower(power);
        }
        else {
            extr.setPower(0.1);
        }
    }
    public void setExtLeftPower(double power){
        power = -power;
        if(ch1.getState()){
            extl.setPower(power);
        }else if(power>0){
            extl.setPower(power);
        }
        else {
            extl.setPower(0.1);
        }

    }
    public double getZnak(double a){
        if(a > 0){
            return 1;
        }
        else if(a == 0){
            return 0;
        }
        else {
            return -1;
        }
    }

}





