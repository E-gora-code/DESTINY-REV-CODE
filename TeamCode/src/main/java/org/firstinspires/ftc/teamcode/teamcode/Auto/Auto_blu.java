package org.firstinspires.ftc.teamcode.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teamcode.openCV.Detector;
import org.firstinspires.ftc.teamcode.teamcode.openCV.SecondaryColorDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Auto_blu", group = "Autonomous")
public class Auto_blu extends OpMode {

    // Hardware devi
    // ces
    ElapsedTime moveTimer = new ElapsedTime();
    private DcMotor FL, BL, FR, BR;
    private Servo sbkr, grabr, grabl, sbros;
    private DcMotor extl, extr;
    private BNO055IMU Gyro;
    private OpenCvCamera webcam;
    private SecondaryColorDetector detector;

    // Pedro Pathing
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // Positions (in cm, converted to inches for Pedro)
    private final Pose startPose = new Pose(144, 60, Math.toRadians(-90)); // Facing -90 degrees (down)
    private final Pose chekPose1 = new Pose(144-7,124,Math.toRadians(-90));
    private final Pose chekPose2 = new Pose(144-28,124,Math.toRadians(-90));
    private final Pose chekPose3 = new Pose(144-52,120,Math.toRadians(-90));
    private final Pose apple= new Pose(144-19.412, 62.961,Math.toRadians(-180));
    private final Pose factory= new Pose(144-21.419, 35.805, Math.toRadians(-270));


    // Paths
    private PathChain moveToScore1;
    private PathChain moveToScore2;
    private PathChain moveToScore3;
    private PathChain  moveToScoreg1;
    private PathChain  moveToScoreg2;
    private PathChain  moveToScoreg3;
    private Path park;
    private PathChain score1;
    private PathChain score2;
    private PathChain score3;
    private PathChain movetoredbox1;
    private PathChain movetoap1;
    private PathChain movetoap2;
    private PathChain movetoap3;
    private PathChain moveTofac;
    private PathChain movetofactory;

    // Other
    private double Base = 3;
    private double Red = 2;
    private double Grenn = 3;

    Telemetry dash = FtcDashboard.getInstance().getTelemetry();

    @Override
    public void init() {
        sbkr = hardwareMap.servo.get("sbkr");
        sbros = hardwareMap.servo.get("sbros");
        grabl = hardwareMap.servo.get("grabl");
        grabr = hardwareMap.servo.get("grabr");

        extl = hardwareMap.dcMotor.get("extl");
        extr = hardwareMap.dcMotor.get("extr");

        extl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        extr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // В вашем основном классе:


// В методе init():
        detector = new SecondaryColorDetector();
        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        FtcDashboard.getInstance().startCameraStream(webcam, 24);

        // Initialize Pedro Pathing
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose((startPose));
        // Build paths
        buildPaths();
    }

    @Override
    public void init_loop() {
        // Camera telemetry while waiting for start
        telemetry.addData("Base", Base);
        dash.addData("fill1", detector.fill1);
        dash.addData("center1", detector.centerX1);
        dash.addData("fill2", detector.fill2);
        dash.addData("center2", detector.centerX2);
        dash.update();

        if (detector.fill2 < 0.00003) {
            Grenn = 3;
            dash.addData("parkGREEN", "3");
        } else if (detector.centerX2 > 200) {
            Grenn = 2;
            dash.addData("parkGREEN", "2");
        } else if (detector.centerX2 < 200) {
            Grenn  = 1;
            dash.addData("parkGREEN", "1");
        }
        if (detector.fill1 < 0.00003) {
            Red = 3;
            dash.addData("parkRED", "3");
        } else if (detector.centerX1 > 200) {
            Red= 2;
            dash.addData("parkRED", "2");
        } else if (detector.centerX1 < 200) {
            Red = 1;
            dash.addData("parkRED", "1");
        }
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        webcam.stopStreaming();
    }

    @Override
    public void loop() {
        autonomousPathUpdate();
        follower.update();

        // Telemetry
        Pose currentPoseInInches = follower.getPose();
        Pose currentPoseInCm = new Pose(
                currentPoseInInches.getX() * 2.54,
                currentPoseInInches.getY() * 2.54,
                currentPoseInInches.getHeading()
        );

        telemetry.addData("Path State", pathState);
        telemetry.addData("X (cm)", currentPoseInCm.getX());
        dash.addData("X (cm)", currentPoseInCm.getX());
        dash.addData("Y (cm)", currentPoseInCm.getY());
        telemetry.addData("Y (cm)", currentPoseInCm.getY());
        telemetry.addData("Heading", Math.toDegrees(currentPoseInCm.getHeading()));
        dash.update();
        telemetry.update();
    }

    @Override
    public void stop() {
        webcam.stopStreaming();
    }
    private void buildPaths() {
        // Путь с разворотом на 180 градусов в конце


        moveToScore1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(startPose),
                                new Point(-4.227, 59.973, Point.CARTESIAN),

                                new Point(chekPose1)
                        )
                )
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        chekPose1.getHeading()
                )
                .build();
        movetoap1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(chekPose1),
                                new Point(144-9.227, 59.973, Point.CARTESIAN),
                                new Point(apple)
                        )
                )
                .setLinearHeadingInterpolation(
                        chekPose1.getHeading(),  // Начальный угол
                        apple.getHeading() // Конечный угол (разворот на 180°)
                )
                .build();
        moveToScore2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(startPose),
                                new Point(144-28.0,70),
                                new Point(chekPose2)
                        )
                )
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        chekPose2.getHeading()
                )
                .build();
        movetoap2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(chekPose2),
                                new Point(144-9.227, 59.973, Point.CARTESIAN),
                                new Point(apple)
                        )
                )
                .setLinearHeadingInterpolation(
                        chekPose2.getHeading(),  // Начальный угол
                        apple.getHeading() // Конечный угол (разворот на 180°)
                )
                .build();
        moveToScore3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(startPose),
                                new Point(144-3.130, 105.446, Point.CARTESIAN),
                                new Point(144-56.130, 109.446, Point.CARTESIAN),
                                new Point(chekPose3)
                        )
                )
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        chekPose3.getHeading()
                )
                .build();
        movetoap3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(chekPose3),
                                new Point(144-0, 100.849, Point.CARTESIAN),
                                new Point(144-0, 67.849, Point.CARTESIAN),
                                new Point(apple)
                        )
                )
                .setLinearHeadingInterpolation(
                        chekPose3.getHeading(),  // Начальный угол
                        apple.getHeading() // Конечный угол (разворот на 180°)
                )
                .build();

        moveTofac= follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(apple),
                                new Point(factory)
                        )
                )
                .setLinearHeadingInterpolation(
                        apple.getHeading(),factory.getHeading()
                )
                .build();
        moveToScoreg1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(factory),
                                new Point(chekPose1)
                        )
                )
                .setLinearHeadingInterpolation(
                        factory.getHeading(),
                        chekPose1.getHeading()
                )
                .build();
        moveToScoreg2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(factory),
                                new Point(144-28.0,70),
                                new Point(chekPose2)
                        )
                )
                .setLinearHeadingInterpolation(
                        factory.getHeading(),
                        chekPose1.getHeading()
                )
                .build();
        moveToScoreg3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(factory),
                                new Point(144-3.130, 105.446, Point.CARTESIAN),
                                new Point(144-55.130, 106.446, Point.CARTESIAN),
                                new Point(chekPose3)
                        )
                )
                .setLinearHeadingInterpolation(
                        factory.getHeading(),
                        chekPose1.getHeading()
                )
                .build();


    }



    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                moveTimer.reset();

                if (Red==1){
                    setPathState(1);
                }
                else if (Red==2){
                    setPathState(3);
                }
                else{
                    setPathState(5);
                }
                break;
            case 1: // Move to first position
                follower.followPath(moveToScore1,true);
                setPathState(2);
                break;

            case 2:
                if (moveTimer.milliseconds()>2000){
                    extl.setPower(0);
                    extr.setPower(0);


                }// Move to AprilTag position 1
                if (!follower.isBusy()) {
                    grabr.setPosition(0.05);
                    grabl.setPosition(0.75);
                    sleep(1000);


                    follower.followPath(movetoap1,true);
                    setPathState(7);
                }
                break;
            case 3:
                follower.followPath(moveToScore2,true);
                setPathState(4);
                break;
            case 4:
                if (moveTimer.milliseconds()>2000){
                    extl.setPower(0);
                    extr.setPower(0);


                }
                if (!follower.isBusy()) {
                    grabr.setPosition(0.05);
                    grabl.setPosition(0.75);
                    sleep(1000);
                    follower.followPath(movetoap2,true);
                    setPathState(7);
                }
                break;
            case 5:

                follower.followPath(moveToScore3,true);
                setPathState(6);
                break;
            case 6:
                if (moveTimer.milliseconds()>2500){
                    extl.setPower(0);
                    extr.setPower(0);



                }
                dash.addData("ext",extl.getCurrentPosition());
                dash.update();
                if (!follower.isBusy()) {
                    grabr.setPosition(0.05);
                    grabl.setPosition(0.75);
                    sleep(1000);
                    follower.followPath(movetoap3,true);
                    setPathState(7);
                }
                break;

            case 7: // Final park
                if (!follower.isBusy()) {
                    extl.setPower(-1);
                    extr.setPower(1);
                    follower.followPath(moveTofac,true);
                    setPathState(8);
                }
                break;
            case 8:

                if (Grenn ==1){
                    setPathState(9);
                }
                else if (Grenn ==2){
                    setPathState(11);
                }
                else{
                    setPathState(13);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    grabr.setPosition(1);
                    grabl.setPosition(0);
                    sleep(1000);// Move to first position
                    follower.followPath(moveToScoreg1,true);
                    setPathState(10);}
                break;

            case 10: // Move to AprilTag position 1
                if (!follower.isBusy()) {
                    grabr.setPosition(0.05);
                    grabl.setPosition(0.75);
                    sleep(1000);
                    follower.followPath(movetoap1,true);
                    setPathState(15);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    grabr.setPosition(1);
                    grabl.setPosition(0);
                    sleep(1000);
                    follower.followPath(moveToScoreg2,true);
                    setPathState(12);}
                break;
            case 12:
                if (!follower.isBusy()) {
                    grabr.setPosition(0.05);
                    grabl.setPosition(0.75);
                    sleep(1000);
                    follower.followPath(movetoap2,true);
                    setPathState(15);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    grabr.setPosition(1);
                    grabl.setPosition(0);
                    sleep(1000);
                    follower.followPath(moveToScoreg3,true);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    grabr.setPosition(0.05);
                    grabl.setPosition(0.75);
                    sleep(1000);
                    follower.followPath(movetoap3,true);
                    setPathState(15);
                }
                break;

            case 15: // Final park
                if (!follower.isBusy()) {
                    sbros.setPosition(1);
                    sleep(400);
                    sbros.setPosition(0.5);
                    sleep(400);
                    sbros.setPosition(1);
                    sleep(400);
                    sbros.setPosition(0.5);
                    sleep(400);
                    sbros.setPosition(1);
                    sleep(400);
                    sbros.setPosition(0.5);
                    sleep(400);

                    follower.followPath(moveTofac,true);
                    setPathState(16);
                }
                break;







            case 16: // Final park
                if (!follower.isBusy()) {
                    grabr.setPosition(1);
                    grabl.setPosition(0);
                    sleep(1000);

                    setPathState(-1); // End
                }
                break;

            case -1: // End
                break;
        }
    }

    private void operateExtension() {
        extl.setPower(-0.7);
        extr.setPower(0.7);
        sleep(1000);

        extl.setPower(0.7);
        extr.setPower(-0.7);
        sleep(1000);

        extl.setPower(0);
        extr.setPower(0);
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }}