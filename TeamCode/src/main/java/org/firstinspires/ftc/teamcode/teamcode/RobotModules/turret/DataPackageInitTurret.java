package org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.generic_classes.RobotHardware;

public class DataPackageInitTurret {
    public HardwareMap TEMP_HardwareMap;
    public RobotHardware.Motors.BasicServo yaw;
    public RobotHardware.Motors.BasicServo pitch;

    public RobotHardware.Motors.DCMotor shooterL;
    public RobotHardware.Motors.DCMotor shooterR;

    public DataPackageInitTurret(HardwareMap hardware){
        this.TEMP_HardwareMap =hardware;
    }
}
