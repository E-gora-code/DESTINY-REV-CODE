package org.firstinspires.ftc.teamcode.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.teamcode.generic_classes.OpModeFramework;
import org.firstinspires.ftc.teamcode.teamcode.generic_classes.RobotHardware;

import java.util.Random;


@TeleOp
public class hardvaretest extends OpModeFramework {

    tred_1 tred__1 = new tred_1();

    double var1 = 0;
    Random random = new Random();
    Telemetry dash = FtcDashboard.getInstance().getTelemetry();

//    @Override
    public void runOpMode() throws InterruptedException {
        selfInit();


        gyro.init_all();
        driveBase.init_all();
        servoMotors.init_all(true);




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
        tred__1.start();
        while (opModeIsActive()) {
//            driveBase.FR = 1;
            FL.setPower(0);
            driveBase.send_to_motors();
            servoMotors.send_to_components();

        }

    }
    public void Tel(){
        telemetry.addData("gyro",gyro.Angle().firstAngle);
        telemetry.update();
        dash.update();
    }

    class tred_1 extends Thread{
        public void run(){
            while (opModeIsActive()){
                Tel();
                claw.setPosition(random.nextDouble());
            }
        }
    }
}





