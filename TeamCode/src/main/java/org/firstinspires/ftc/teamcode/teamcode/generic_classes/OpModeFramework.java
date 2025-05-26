package org.firstinspires.ftc.teamcode.teamcode.generic_classes;

import org.firstinspires.ftc.teamcode.teamcode.generic_classes.RobotHardware;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.teamcode.PID_setting;
import org.firstinspires.ftc.teamcode.teamcode.openCV.CameraOverlay;
import org.firstinspires.ftc.teamcode.teamcode.openCV.Detector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


public abstract class OpModeFramework extends LinearOpMode {
    protected RobotHardware.DriveBase.motor_classes.FrontLeft FL;
    protected RobotHardware.DriveBase.motor_classes.FrontRight FR;
    protected RobotHardware.DriveBase.motor_classes.BackLeft BL;
    protected RobotHardware.DriveBase.motor_classes.BackRight BR;

    protected RobotHardware.ServoMotors.BasicServo sbros, claw, grabr, grabl;
    protected RobotHardware.ServoMotors.SparkMotor extr, extl;
    protected RobotHardware robotHardware;
    protected RobotHardware.ServoMotors servoMotors;
    protected RobotHardware.DriveBase driveBase;
    protected RobotHardware.GyroIMU gyro;
    protected RobotHardware.DriveBase.motor_classes motor_classes;

    DigitalChannel ch0, ch1;

    public void selfInit(){
        robotHardware = new RobotHardware(hardwareMap);
        gyro = robotHardware.new GyroIMU();
        driveBase = robotHardware.new DriveBase();
        motor_classes = driveBase.new motor_classes();

        FL = motor_classes.new FrontLeft(driveBase);
        FR = motor_classes.new FrontRight(driveBase);
        BL = motor_classes.new BackLeft(driveBase);
        BR = motor_classes.new BackRight(driveBase);

        servoMotors = robotHardware.new ServoMotors();

        sbros = servoMotors.new BasicServo(servoMotors, RobotHardware.ServoMotors.servoKeys.apple_drop_module);
        claw = servoMotors.new BasicServo(servoMotors, RobotHardware.ServoMotors.servoKeys.hidden_claw_module);
        grabl = servoMotors.new BasicServo(servoMotors, RobotHardware.ServoMotors.servoKeys.container_grab_module.leftServo);
        grabr = servoMotors.new BasicServo(servoMotors, RobotHardware.ServoMotors.servoKeys.container_grab_module.rightServo);

        extr = servoMotors.new SparkMotor(servoMotors,RobotHardware.ServoMotors.servoKeys.SparkMiniKeys.extention_right);
        extl = servoMotors.new SparkMotor(servoMotors,RobotHardware.ServoMotors.servoKeys.SparkMiniKeys.extention_left);

        // FIXME: 26.05.2025 maybe make a class for these
        ch0 = hardwareMap.digitalChannel.get("0");
        ch1 = hardwareMap.digitalChannel.get("1");
        ch0.setMode(DigitalChannel.Mode.INPUT);
        ch1.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public abstract void runOpMode() throws InterruptedException;
}
