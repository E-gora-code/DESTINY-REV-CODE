package org.firstinspires.ftc.teamcode.teamcode.openCV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//@Autonomous
public class OpenCVCamera extends LinearOpMode {
    OpenCvCamera webcam;
    HSV hsv = new HSV();
    Telemetry dash = FtcDashboard.getInstance().getTelemetry();
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        Detector detector = new Detector();
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });
        FtcDashboard.getInstance().startCameraStream(webcam, 24);
        while(!opModeIsActive()){
            dash.addData("fill", detector.fillValue);
            dash.addData("center",detector.center);
            dash.update();
            if (detector.fillValue < 0.03) dash.addData("park", "3");
            else if (detector.center < 100) dash.addData("park", "1");
            else if (detector.center > 100) dash.addData("park", "2");
            detector.setHSV(hsv.lh, hsv.ls, hsv.lv, hsv.hh, hsv.hs, hsv.hv);
             // Первые значения
        }
        waitForStart();

        webcam.stopStreaming();
    }
}
