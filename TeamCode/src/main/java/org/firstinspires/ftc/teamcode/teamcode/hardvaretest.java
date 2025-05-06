package org.firstinspires.ftc.teamcode.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.teamcode.generic_classes.RobotHardware;



@TeleOp
public class
hardvaretest extends LinearOpMode {
    RobotHardware robotHardvare = new RobotHardware();
    RobotHardware.DriveBase driveBase = robotHardvare.new DriveBase();

    tred_1 tred__1 = new tred_1();
    Servo sbkr, grabr, grabl, sbros;
    CRServo extl, extr;


    Orientation Orientation = new Orientation();
    Acceleration Acceleration = new Acceleration();
    DigitalChannel ch0, ch1;
    double var1 = 0;
    Telemetry dash = FtcDashboard.getInstance().getTelemetry();

//    @Override
    public void runOpMode() throws InterruptedException {

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
        driveBase.init_all();




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
            driveBase.FR = 1;
            driveBase.send_to_motors();

        }

    }
    public void Tel(){
        telemetry.update();
        dash.update();
    }

    class tred_1 extends Thread{
        public void run(){
            while (opModeIsActive()){
                Tel();
            }
        }
    }
    // 8===========D
}





