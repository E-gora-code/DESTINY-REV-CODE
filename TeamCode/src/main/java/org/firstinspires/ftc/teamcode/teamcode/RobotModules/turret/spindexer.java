package org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.generic_classes.RobotHardware;

public class spindexer {
    private RobotHardware.Motors.BasicServo Front_ejector,Back_ejector,Back_wall,Front_wall;
    public RobotHardware.Motors.BasicServo spindexer;
    private RobotHardware.Motors.DCMotor Front_intake,Back_intake;

    public spindexer(DataPackageInitSpindexer pack) {
        this.spindexer = pack.spindexer;
        this.Front_ejector = pack.Front_ejector;
        this.Back_ejector = pack.Back_ejector;
        this.Back_wall = pack.Back_wall;
        this.Front_wall = pack.Front_wall;

    }
    public double getSpindexerPosition(){
        return  spindexer.getEncoderPosition();
    }

    public void update(boolean shoot,boolean ready,boolean intaking) {
        if (intaking){
            Front_intake.setPower(1);
            Back_intake.setPower(1);
        }
        if (ready){
            Front_ejector.setPosition(config.Front_ejector_noshoot_position);
            Back_ejector.setPosition(config.Back_ejector_shoot_position);
            spindexer.setPower(1);
            Front_intake.setPower(1);
            Back_intake.setPower(1);
        }
        else{
            Front_ejector.setPosition(config.Front_ejector_noshoot_position);
            Back_ejector.setPosition(config.Back_ejector_noshoot_position);
        }
        if (shoot){
            Back_wall.setPosition(config.Back_wall_spin_position);
            Front_wall.setPosition(config.Front_wall_spin_position);
        }
        else{
            Back_wall.setPosition(config.Back_wall_nospin_position);
            Front_wall.setPosition(config.Front_wall_nospin_position);
        }
    }
}