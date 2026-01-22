package org.firstinspires.ftc.teamcode.teamcode.control;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.generic_classes.CucleHelper;
import org.firstinspires.ftc.teamcode.generic_classes.GamepadDriver;
import org.firstinspires.ftc.teamcode.generic_classes.OpModeFramework;
import org.firstinspires.ftc.teamcode.generic_classes.OpRobotSystemsFramework;
import org.firstinspires.ftc.teamcode.generic_classes.ToggleHelper;

@TeleOp
public class Manual extends OpRobotSystemsFramework {

    double extr_pos, extl_pos;
    double Multiply = 0.6;
    boolean b_press = false;
    boolean grab_toggle = false;

    boolean independent_drive = false;
    boolean independent_drive_press = false;

    double turred_y_pos = 0;


    Telemetry dash = FtcDashboard.getInstance().getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        selfInit();
        initAllSystems();
        motors.is_motors_enabled = false;
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
            printTelemetry("Spin_vol", spindexer.getEncoderVoltage());
            printTelemetry("Spin_pos", spindexer.getEncoderPosition());
//            telemetry.addData("spindexer", spindexer.getEncoderPosition());
//            telemetry.addData("spindexer_pos", spindexer.getSentPosition());
//            telemetry.addData("front_ejector_pos", front_ejector.getSentPosition());
//            telemetry.addData("back_ejector_pos", back_ejector.getSentPosition());
//            telemetry.addData("front_wall_pos", front_wall.getSentPosition());
//            telemetry.addData("back_wall_pos", back_wall.getSentPosition());
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
            telemetry.addData("cv0", cv0.value());
            telemetry.addData("cv1", cv1.value());
            telemetry.addData("cv0r", cv0.red());
            telemetry.addData("cv0g", cv0.green());
            telemetry.addData("cv0b", cv0.blue());
            telemetry.addData("cv0COL", spindexerModule.checkColor());
            telemetry.addData("pose", spindexerModule.input_count);
//            telemetry.addData("extR", extr_pos);
//            telemetry.addData("extL", extl_pos);
//            telemetry.addData("BL", BL.getCurrentPosition());
//            telemetry.addData("BR", BR.getCurrentPosition());
//            telemetry.addData("Start_press", (gamepad1.start || gamepad2.start));
//            telemetry.addData("A_press", (gamepad1.a || gamepad2.a));
//            dash.addData("angle",currentAngle);
//            dash.addData("angletarg",targAngle);
//            dash.addData("Angle", Angle());
            telemetry.update();
//            dash.addData("x", BR.getCurrentPosition());
//            dash.addData("y", BL.getCurrentPosition());
            dash.update();

            spindexerModule.enabled = gamepad2.dpad_down;

            spindexerModule.front_intaking = gamepad2.dpad_down||gamepad1.dpad_down;
            spindexerModule.front_shoot = gamepad2.dpad_left||gamepad1.dpad_left;
            spindexerModule.spin = gamepad2.right_stick_y+gamepad1.right_stick_y;
            if(Math.abs(controllerDriver_2.internal_touchpad.getY())<0.1) {
                turret_x.setPower(controllerDriver_2.internal_touchpad.getX() + gamepad2.left_stick_y);
            }
            turret_y.setPosition(controllerDriver_2.internal_touchpad.getY()/3);


            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                gamepad1.rumble(3000);
                gamepad2.rumble(3000);

            }


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





