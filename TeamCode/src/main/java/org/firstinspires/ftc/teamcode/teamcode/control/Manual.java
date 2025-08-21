package org.firstinspires.ftc.teamcode.teamcode.control;


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
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.teamcode.PID_setting;


@TeleOp
public class Manual extends LinearOpMode {
    boolean xkfi = false;
    PID_setting pid_setting = new PID_setting();
    double extr_zero = 0, extl_zero = 0;
    double extr_max = 5170, extl_max;
    DcMotor FL, BL, FR, BR;
    Servo sbkr, grabr, grabl, sbros,vilas,vilar;
    DcMotor extl, extr;
    double currentAngle = 0;
    double exterR, exterL, extpowerR, extpowerL, extrlR, extrlL;
    double extr_pos, extl_pos;
    double Multiply = 0.6;
    double turnPower = 0;
    double targAngle = 0;
    double turnErr = 0, turnErrL = 0;
    double deltaHed = 0, deltaHedL = 0;
    double ext_pos_calk = 0;
    double gamepad_summ = 0;
    boolean lastext = false;
    boolean presed_reset_ang = false;
    boolean ext_press = false;
    boolean is_homed_ever = false;
    boolean b_press = false;
    boolean grab_toggle = false;

    boolean independent_drive = false;
    boolean independent_drive_press = false;
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


    ElapsedTime extention_time = new ElapsedTime();
    double extention_time_interval = 10;

    double color_pulse = 1;
    ElapsedTime color_pulse_timer = new ElapsedTime();


    boolean can_homa_wake_up = true;

    boolean ishoma = false;
    ElapsedTime lastcl = new ElapsedTime();
    boolean iscl1 = false;
    boolean iscl = false;
    BNO055IMU Gyro;
    Orientation Orientation = new Orientation();
    Acceleration Acceleration = new Acceleration();
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
        extl = hardwareMap.dcMotor.get("extl");
        extr = hardwareMap.dcMotor.get("extr");

        extl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

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
//        });

        waitForStart();
        while (opModeIsActive()) {
            // varing
            extr_pos = FL.getCurrentPosition();
            extl_pos = FR.getCurrentPosition();
            telemetry.addData("alt",independent_drive);
            telemetry.addData("Angle X", Angle().firstAngle);
            telemetry.addData("Angle Y", Angle().secondAngle);
            telemetry.addData("Angle Z", Angle().thirdAngle);
            telemetry.addData("Acceliration", (Axel().xAccel+Axel().yAccel+Axel().zAccel));

            telemetry.addData("ch0", ch0.getState());
            telemetry.addData("ch1", ch1.getState());
            telemetry.addData("extR",extr.getCurrentPosition() );
            telemetry.addData("extL", extl.getCurrentPosition());
            telemetry.addData("BL", BL.getCurrentPosition());
            telemetry.addData("BR", BR.getCurrentPosition());
            telemetry.addData("Start_press",(gamepad1.start||gamepad2.start));
            telemetry.addData("A_press",(gamepad1.a||gamepad2.a));
//            dash.addData("angle",currentAngle);
//            dash.addData("angletarg",targAngle);
//            dash.addData("Angle", Angle());
            telemetry.update();
            dash.addData("x",BR.getCurrentPosition());
            dash.addData("y",BL.getCurrentPosition());
            dash.update();


//            if (gamepad2.right_trigger > 0) {
//                sbros.setPosition(gamepad2.right_trigger + 0);
//            }
//            if (gamepad1.right_bumper) {
//                sbros.setPosition(gamepad1.right_trigger + 0);
//            }


            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                gamepad1.rumble(3000);
                gamepad2.rumble(3000);

            }


            if (gamepad1.a || gamepad2.right_bumper) {
                sbkr.setPosition(1);
            } else {
                if (gamepad1.right_bumper != true) {
                    sbkr.setPosition((gamepad1.right_trigger * 0.48) + 0.52);
                }

            }

            if (gamepad1.b) {
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

            if((gamepad2.left_stick_y!=0)&&(gamepad2.right_stick_x!=0)){
                extl.setPower(-gamepad2.right_stick_y);
                extr.setPower(-gamepad2.right_stick_y);
            }else {
                extl.setPower(-gamepad2.left_stick_y);
                extr.setPower(-gamepad2.right_stick_y);
            }


//        else{
//            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            pos = 0;
//        }
            if(gamepad1.dpad_up){
                if(independent_drive_press==false){
                    independent_drive_press = true;
                    independent_drive = !independent_drive;
                }
            }
            else{
                independent_drive_press = false;
            }
            if(independent_drive==false) {
                BR.setPower((-gamepad1.left_stick_x + gamepad1.right_stick_x + gamepad1.left_stick_y) * Multiply);
                BL.setPower((-gamepad1.left_stick_x + gamepad1.right_stick_x - gamepad1.left_stick_y) * Multiply);
                FR.setPower((gamepad1.left_stick_x + gamepad1.right_stick_x + gamepad1.left_stick_y) * Multiply*0.7);
                FL.setPower((gamepad1.left_stick_x + gamepad1.right_stick_x - gamepad1.left_stick_y) * Multiply);
            }
            else {
                FL.setPower(-gamepad1.left_stick_y);
                FR.setPower(gamepad1.right_stick_y*0.7);
                BL.setPower(gamepad1.left_stick_x);
                BR.setPower(gamepad1.right_stick_x);
            }

//            BR.setPower((gamepad1.left_stick_x-gamepad1.left_stick_y+turnPower)*Multiply);
//            BL.setPower((-gamepad1.left_stick_x-gamepad1.left_stick_y-turnPower)*Multiply);
//            FR.setPower((-gamepad1.left_stick_x-gamepad1.left_stick_y+turnPower)*Multiply);
//            FL.setPower((gamepad1.left_stick_x-gamepad1.left_stick_y-turnPower)*Multiply);


        }

    }

    public Orientation Angle() {
        Orientation = Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Orientation;
    }

    public Acceleration Axel() {
        Acceleration = Gyro.getAcceleration();
        return Acceleration;
    }

}





