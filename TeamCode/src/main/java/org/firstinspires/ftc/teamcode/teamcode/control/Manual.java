package org.firstinspires.ftc.teamcode.teamcode.control;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.generic_classes.GamepadDriver;
import org.firstinspires.ftc.teamcode.generic_classes.OpModeFramework;
import org.firstinspires.ftc.teamcode.generic_classes.ToggleHelper;

@TeleOp
public class Manual extends OpModeFramework {

    double extr_pos, extl_pos;
    double Multiply = 0.6;
    boolean b_press = false;
    boolean grab_toggle = false;

    boolean independent_drive = false;
    boolean independent_drive_press = false;

    public void front_up(){
        front_wall.setPosition(1);
    }
    public void front_down(){
        front_wall.setPosition(0);
    }
    ToggleHelper front_wall_toggle = new ToggleHelper(this::front_up,this::front_down,true);
//    Servo s1;

    Telemetry dash = FtcDashboard.getInstance().getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        selfInit();
        initAllSystems();
//          s1 = hardwareMap.servo.get("s1");
////        s2 = hardwareMap.servo.get("servo3");
//
//

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
//             varing
            extr_pos = FL.getCurrentPosition();
            extl_pos = FR.getCurrentPosition();
            telemetry.addData("spindexer", spindexer.getEncoderPosition());
            telemetry.addData("spindexer_pos", spindexer.getSentPosition());
            telemetry.addData("front_ejector_pos", front_ejector.getSentPosition());
            telemetry.addData("back_ejector_pos", back_ejector.getSentPosition());
            telemetry.addData("front_wall_pos", front_wall.getSentPosition());
            telemetry.addData("back_wall_pos", back_wall.getSentPosition());
            telemetry.addData("alt", independent_drive);
            telemetry.addData("PS5", gamepad2.touchpad_finger_1);
            telemetry.addData("PSx", controllerDriver_2.internal_touchpad.getX());
            telemetry.addData("PSy", controllerDriver_2.internal_touchpad.getY());
            telemetry.addData("Angle X", gyro.Angle().firstAngle);
            telemetry.addData("Angle Y", gyro.Angle().secondAngle);
            telemetry.addData("Angle Z", gyro.Angle().thirdAngle);
            telemetry.addData("Acceliration", (gyro.Axel().xAccel + gyro.Axel().yAccel + gyro.Axel().zAccel));

            telemetry.addData("ch0", ch0.getState());
            telemetry.addData("ch1", ch1.getState());
//            telemetry.addData("extR", extr_pos);
//            telemetry.addData("extL", extl_pos);
//            telemetry.addData("BL", BL.getCurrentPosition());
//            telemetry.addData("BR", BR.getCurrentPosition());
            telemetry.addData("Start_press", (gamepad1.start || gamepad2.start));
            telemetry.addData("A_press", (gamepad1.a || gamepad2.a));
//            dash.addData("angle",currentAngle);
//            dash.addData("angletarg",targAngle);
//            dash.addData("Angle", Angle());
            telemetry.update();
            dash.addData("x", BR.getCurrentPosition());
            dash.addData("y", BL.getCurrentPosition());
            dash.update();




            spindexer.setPower((-gamepad2.left_trigger+gamepad2.right_trigger)*0.5);
//            front_wall.setPosition(1-gamepad2.right_stick_y);
            back_wall.setPosition(gamepad2.left_stick_y);
            back_ejector.setPosition(1-gamepad2.right_stick_x);

            front_wall_toggle.acceptIn(gamepad2.a);

            turret_x.setPower(controllerDriver_2.internal_touchpad.touchpad1_X);

            front_ejector.setPosition(gamepad2.left_stick_x);
            double power_of_intake = 0.6;
            if(gamepad2.dpad_right) {
                front_intake.setPower(power_of_intake);
            }else if(gamepad2.dpad_left){
                front_intake.setPower(-power_of_intake);
            }else{
                front_intake.setPower(0);
            }
            if(gamepad2.dpad_up) {
                back_intake.setPower(power_of_intake);
            }else if(gamepad2.dpad_down){
                back_intake.setPower(-power_of_intake);
            }else{
                back_intake.setPower(0);
            }
            if(gamepad2.left_bumper){
                shooter_right.setPower(-1);
                shooter_left.setPower(1);
            }else {
                shooter_right.setPower(0);
                shooter_left.setPower(0);
            }


            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                gamepad1.rumble(3000);
                gamepad2.rumble(3000);

            }
//
//
//
//
//            i
//
//            i
//
//            if(gamepad2.dpad_up){
//                reika.setPower(1);
//            }
//            else if(gamepad2.dpad_down){
//                reika.setPower(-1);
//            }
//            else {
//                reika.setPower(0);
//            }
//            ;
//

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
//
    }




       }





