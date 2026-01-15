package org.firstinspires.ftc.teamcode.generic_classes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public abstract class OpModeFramework extends LinearOpMode {
    protected Telemetry dash;


    protected RobotHardware.DriveBase.motor_classes.FrontLeft FL;
    protected RobotHardware.DriveBase.motor_classes.FrontRight FR;
    protected RobotHardware.DriveBase.motor_classes.BackLeft BL;
    protected RobotHardware.DriveBase.motor_classes.BackRight BR;

    protected RobotHardware.Motors.BasicServo sbros, claw, grabr, grabl,reika, vila_r, vila_l;
    protected RobotHardware.Motors.BasicServo spindexer,front_ejector,back_ejector,front_wall,back_wall;
    protected RobotHardware.Motors.DCMotor extr, extl,factory_ext;

    protected RobotHardware robotHardware;
    protected GamepadDriver controllerDriver_1;
    protected GamepadDriver controllerDriver_2;
    protected RobotHardware.Motors motors;
    protected RobotHardware.Sensors sensors;
    protected RobotHardware.DriveBase driveBase;
    protected RobotHardware.GyroIMU gyro;
    protected RobotHardware.DriveBase.motor_classes motor_classes;

    protected RobotHardware.Sensors.BasicChannel ch0, ch1;

    public void selfInit(){
        //Error processing settings
        robotHardware = new RobotHardware(hardwareMap,RobotHardware.errorResponses::ignore);

        controllerDriver_1 = new GamepadDriver(gamepad1);
        controllerDriver_2 = new GamepadDriver(gamepad2);

        gyro = robotHardware.new GyroIMU();
        driveBase = robotHardware.new DriveBase();
        motor_classes = driveBase.new motor_classes();

        FL = motor_classes.new FrontLeft(driveBase);
        FR = motor_classes.new FrontRight(driveBase);
        BL = motor_classes.new BackLeft(driveBase);
        BR = motor_classes.new BackRight(driveBase);

        dash = FtcDashboard.getInstance().getTelemetry();

        motors = robotHardware.new Motors();
        sensors = robotHardware.new Sensors();

        spindexer = motors.new BasicServo(motors, RobotHardware.Motors.NameKeys.servoNameKeys.spindexer);
        front_ejector = motors.new BasicServo(motors, RobotHardware.Motors.NameKeys.servoNameKeys.front_ejector);
        back_ejector = motors.new BasicServo(motors, RobotHardware.Motors.NameKeys.servoNameKeys.back_ejector);
        front_wall = motors.new BasicServo(motors, RobotHardware.Motors.NameKeys.servoNameKeys.front_wall);
        back_wall = motors.new BasicServo(motors, RobotHardware.Motors.NameKeys.servoNameKeys.back_wall);



        claw = motors.new BasicServo(motors, RobotHardware.Motors.NameKeys.servoNameKeys.hidden_claw_module);
        grabl = motors.new BasicServo(motors, RobotHardware.Motors.NameKeys.servoNameKeys.container_grab_module.leftServo);
        grabr = motors.new BasicServo(motors, RobotHardware.Motors.NameKeys.servoNameKeys.container_grab_module.rightServo);

        vila_r = motors.new BasicServo(motors, RobotHardware.Motors.NameKeys.servoNameKeys.fork_module.fork_right);
        vila_l = motors.new BasicServo(motors, RobotHardware.Motors.NameKeys.servoNameKeys.fork_module.fork_left);

        reika = motors.new BasicServo(motors, RobotHardware.Motors.NameKeys.servoNameKeys.factory_lift);

        extr = motors.new DCMotor(motors, RobotHardware.Motors.NameKeys.motorDCNameKeys.extention_right);
        extl = motors.new DCMotor(motors, RobotHardware.Motors.NameKeys.motorDCNameKeys.extention_left);

        factory_ext = motors.new DCMotor(motors, RobotHardware.Motors.NameKeys.motorDCNameKeys.factory_extention);


        ch0 = sensors.new BasicChannel(sensors, RobotHardware.Sensors.NameKeys.channelsNameKeys.ch0);
        ch1 = sensors.new BasicChannel(sensors, RobotHardware.Sensors.NameKeys.channelsNameKeys.ch1);

    }

    public void initAllSystems(){
        driveBase.init_all();
        motors.init_all();
        sensors.init_all();
        gyro.init_all();
    }
    public void tickAll(){
        driveBase.class_tick();
        motors.class_tick();
        sensors.class_tick();
    }
    public void printTelemetry(String caption, Object value){
        telemetry.addData(caption,value);
        dash.addData(caption,value);
    }
    public void UpdatePrint(){
        telemetry.update();
        dash.update();
    }

    @Override
    public abstract void runOpMode() throws InterruptedException;
}