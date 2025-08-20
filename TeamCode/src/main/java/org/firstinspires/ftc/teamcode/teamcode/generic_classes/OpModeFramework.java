package org.firstinspires.ftc.teamcode.teamcode.generic_classes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;


public abstract class OpModeFramework extends LinearOpMode {
    protected RobotHardware.DriveBase.motor_classes.FrontLeft FL;
    protected RobotHardware.DriveBase.motor_classes.FrontRight FR;
    protected RobotHardware.DriveBase.motor_classes.BackLeft BL;
    protected RobotHardware.DriveBase.motor_classes.BackRight BR;

    protected RobotHardware.Motors.BasicServo sbros, claw, grabr, grabl;
    protected RobotHardware.Motors.DCMotor extr, extl,factory_ext;
    protected RobotHardware robotHardware;
    protected RobotHardware.Motors motors;
    protected RobotHardware.DriveBase driveBase;
    protected RobotHardware.GyroIMU gyro;
    protected RobotHardware.DriveBase.motor_classes motor_classes;

    protected DigitalChannel ch0, ch1;

    public void selfInit(){
        robotHardware = new RobotHardware(hardwareMap);
        gyro = robotHardware.new GyroIMU();
        driveBase = robotHardware.new DriveBase();
        motor_classes = driveBase.new motor_classes();

        FL = motor_classes.new FrontLeft(driveBase);
        FR = motor_classes.new FrontRight(driveBase);
        BL = motor_classes.new BackLeft(driveBase);
        BR = motor_classes.new BackRight(driveBase);

        motors = robotHardware.new Motors();

        sbros = motors.new BasicServo(motors, RobotHardware.Motors.NameKeys.servoNameKeys.apple_drop_module);
        claw = motors.new BasicServo(motors, RobotHardware.Motors.NameKeys.servoNameKeys.hidden_claw_module);
        grabl = motors.new BasicServo(motors, RobotHardware.Motors.NameKeys.servoNameKeys.container_grab_module.leftServo);
        grabr = motors.new BasicServo(motors, RobotHardware.Motors.NameKeys.servoNameKeys.container_grab_module.rightServo);

        extr = motors.new DCMotor(motors, RobotHardware.Motors.NameKeys.motorDCNameKeys.extention_right);
        extl = motors.new DCMotor(motors, RobotHardware.Motors.NameKeys.motorDCNameKeys.extention_left);

        factory_ext = motors.new DCMotor(motors, RobotHardware.Motors.NameKeys.motorDCNameKeys.factory_extention);

        // FIXME: 26.05.2025 maybe make a class for these
        ch0 = hardwareMap.digitalChannel.get("0");
        ch1 = hardwareMap.digitalChannel.get("1");
        ch0.setMode(DigitalChannel.Mode.INPUT);
        ch1.setMode(DigitalChannel.Mode.INPUT);
    }

    public void initAllSystems(){
        driveBase.init_all();
        motors.init_all();
        gyro.init_all();
    }
    public void tickAll(){
        driveBase.class_tick();
        motors.class_tick();
    }

    @Override
    public abstract void runOpMode() throws InterruptedException;
}
