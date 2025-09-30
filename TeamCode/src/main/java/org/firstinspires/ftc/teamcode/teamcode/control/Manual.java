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

import org.firstinspires.ftc.teamcode.teamcode.generic_classes.OpModeFramework;

@TeleOp
public class Manual extends OpModeFramework {

    double extr_pos, extl_pos;
    double Multiply = 0.6;
    boolean b_press = false;
    boolean grab_toggle = false;

    boolean independent_drive = false;
    boolean independent_drive_press = false;

    Telemetry dash = FtcDashboard.getInstance().getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        selfInit();
        initAllSystems();
//        FL = hardwareMap.dcMotor.get("FL");
//        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        FR = hardwareMap.dcMotor.get("FR");
//        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        BL = hardwareMap.dcMotor.get("BL");
//        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        BR = hardwareMap.dcMotor.get("BR");
//        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        ch0 = hardwareMap.digitalChannel.get("0");
//        ch1 = hardwareMap.digitalChannel.get("1");
//        ch0.setMode(DigitalChannel.Mode.INPUT);
//        ch1.setMode(DigitalChannel.Mode.INPUT);
//
//        sbkr = hardwareMap.servo.get("sbkr");
//        sbros = hardwareMap.servo.get("sbros");
//        grabl = hardwareMap.servo.get("grabl");
//        grabr = hardwareMap.servo.get("grabr");
////        s1 = hardwareMap.servo.get("servo2");
////        s2 = hardwareMap.servo.get("servo3");
//        extl = hardwareMap.crservo.get("extl");
//        extr = hardwareMap.crservo.get("extr");
//
//
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//
//        Gyro = hardwareMap.get(BNO055IMU.class, "imu");
//
//
//        Gyro.initialize(parameters);

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
            telemetry.addData("PS5",gamepad1.touchpad_finger_1);
            telemetry.addData("PSx",gamepad1.touchpad_finger_1_x);
            telemetry.addData("PSy",gamepad1.touchpad_finger_1_y);
            telemetry.addData("Angle X", gyro.Angle().firstAngle);
            telemetry.addData("Angle Y", gyro.Angle().secondAngle);
            telemetry.addData("Angle Z", gyro.Angle().thirdAngle);
            telemetry.addData("Acceliration", (gyro.Axel().xAccel+gyro.Axel().yAccel+gyro.Axel().zAccel));

            telemetry.addData("ch0", ch0.getState());
            telemetry.addData("ch1", ch1.getState());
            telemetry.addData("extR", extr_pos);
            telemetry.addData("extL", extl_pos);
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
                sbros.setPosition(1);
            } else {
                if (gamepad1.right_bumper != true) {
                    sbros.setPosition((gamepad1.right_trigger * 0.48) + 0.63);
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
            if(gamepad2.dpad_right){
                factory_ext.setPower(-1);
            }
            else if(gamepad2.dpad_left){
                factory_ext.setPower(1);
            }
            else {
                factory_ext.setPower(0);
            }

            if(gamepad2.dpad_up){
                reika.setPower(1);
            }
            else if(gamepad2.dpad_down){
                reika.setPower(-1);
            }
            else {
                reika.setPower(0);
            }
            vila_r.setPosition(gamepad2.right_trigger);
            vila_l.setPosition(1-gamepad2.right_trigger);

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
                FR.setPower(gamepad1.right_stick_y);
                BL.setPower(gamepad1.left_stick_x);
                BR.setPower(gamepad1.right_stick_x);
            }

//            BR.setPower((gamepad1.left_stick_x-gamepad1.left_stick_y+turnPower)*Multiply);
//            BL.setPower((-gamepad1.left_stick_x-gamepad1.left_stick_y-turnPower)*Multiply);
//            FR.setPower((-gamepad1.left_stick_x-gamepad1.left_stick_y+turnPower)*Multiply);
//            FL.setPower((gamepad1.left_stick_x-gamepad1.left_stick_y-turnPower)*Multiply);

            tickAll();
        }

    }



}





