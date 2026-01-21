package org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.generic_classes.RobotHardware;
import org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret.config;

public class spindexer {
    private RobotHardware.Motors.BasicServo front_ejector,back_ejector,back_wall,front_wall;
    public RobotHardware.Motors.BasicServo spindexer;
    private RobotHardware.Motors.DCMotor Front_intake,Back_intake;

    public spindexer(DataPackageInitSpindexer pack) {
        this.spindexer = pack.spindexer;
        this.back_ejector = pack.Back_ejector;
        this.front_ejector = pack.Front_ejector;
        this.front_wall = pack.Front_wall;
        this.back_wall = pack.Back_wall;

    }
    public double getSpindexerPosition(){
        return  spindexer.getEncoderPosition();
    }

    public void update(boolean shoot,boolean ready,boolean intaking) {
        front_ejector.setPosition(config.pos_front_ejector);
        back_ejector.setPosition(config.pos_back_ejector);
        back_wall.setPosition(config.pos_back_wall);
        front_wall.setPosition(config.pos_front_wall);
        spindexer.setPosition(config.pos_spindexer);
//        if (intaking){
////            Front_intake.setPower(1);
////            Back_intake.setPower(1);
//        }
//        if (ready){
//            Front_ejector.setPosition(config.Front_ejector_noshoot_position);
//            Back_ejector.setPosition(config.Back_ejector_shoot_position);
//            spindexer.setPower(1);
////            Front_intake.setPower(1);
////            Back_intake.setPower(1);
//        }
//        else{
//            Front_ejector.setPosition(config.Front_ejector_noshoot_position);
//            Back_ejector.setPosition(config.Back_ejector_noshoot_position);
//            spindexer.setPower(0);
//        }
//        if (shoot){
//            Back_wall.setPosition(config.Back_wall_spin_position);
//            Front_wall.setPosition(config.Front_wall_spin_position);
//        }
//        else{
//            Back_wall.setPosition(config.Back_wall_nospin_position);
//            Front_wall.setPosition(config.Front_wall_nospin_position);
//        }
    }
}