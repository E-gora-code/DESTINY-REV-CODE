package org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.generic_classes.RobotHardware;

public class DataPackageInitSpindexer {
    public HardwareMap TEMP_HardwareMap;
    public RobotHardware.Motors.BasicServo spindexer;
    public RobotHardware.Motors.BasicServo Front_ejector,Back_ejector,Back_wall,Front_wall;
    public RobotHardware.Motors.DCMotor Front_intake,Back_intake, Shooter1, Shooter2;
    public RobotHardware.Sensors.BasicColorSensor colorSensor;


    public DataPackageInitSpindexer(HardwareMap hardware){
        this.TEMP_HardwareMap =hardware;
    }
}
