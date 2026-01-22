package org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.generic_classes.RobotHardware;

public class spindexer {
    private RobotHardware.Motors.BasicServo front_ejector,back_ejector,back_wall,front_wall;
    public RobotHardware.Motors.BasicServo spindexer;
    private RobotHardware.Motors.DCMotor Front_intake,Back_intake, Shooter1, Shooter2;

    public boolean front_intaking, back_intaking, back_shoot, front_shoot;
    public double spin , intake_speed = 0.7;

    public spindexer(DataPackageInitSpindexer pack, HardwareMap hw) {
        this.spindexer = pack.spindexer;
        this.back_ejector = pack.Back_ejector;
        this.front_ejector = pack.Front_ejector;
        this.front_wall = pack.Front_wall;
        this.back_wall = pack.Back_wall;
        this.Front_intake = pack.Front_intake;
        this.Back_intake = pack.Back_intake;
        this.Shooter1 = pack.Shooter1;
        this.Shooter2 = pack.Shooter2;

    }
    public double getSpindexerPosition(){
        return  spindexer.getEncoderPosition();
    }

    public void pos_from_config() {
        front_ejector.setPosition(config.pos_front_ejector);
        back_ejector.setPosition(config.pos_back_ejector);
        back_wall.setPosition(config.pos_back_wall);
        front_wall.setPosition(config.pos_front_wall);
        spindexer.setPosition(config.pos_spindexer);
    }
    public void update(){
         if (front_intaking){
             back_ejector.setPosition(0.75);
             back_wall.setPosition(0);
             front_ejector.setPosition(0.8);
             front_wall.setPosition(1);
             Front_intake.setPower(intake_speed);
             Back_intake.setPower(intake_speed);
             spindexer.setPower(spin);
         }
         else if (back_intaking){
             back_ejector.setPosition(0.75);
             back_wall.setPosition(0.8);
             front_ejector.setPosition(1);
             front_wall.setPosition(0);
             Front_intake.setPower(intake_speed);
             Back_intake.setPower(intake_speed);
             spindexer.setPower(spin);
         }
         else if (back_shoot){
             back_ejector.setPosition(1);
             Front_intake.setPower(1);
             Back_intake.setPower(1);
             spindexer.setPower(spin);
         }
         else if (front_shoot){
             front_ejector.setPosition(1);
             Front_intake.setPower(1);
             Back_intake.setPower(1);
             spindexer.setPower(spin);
         }
         else {
             Front_intake.setPower(0);
             Back_intake.setPower(0);
             spindexer.setPower(spin);
         }

    }
    public boolean rotate_to(double pos,double power){
        double enc = spindexer.getEncoderPosition();
        if(Math.abs(pos-enc)>0.2) {
            if (pos - enc >= 0) {
                spindexer.setPower(-power);

            } else {
                spindexer.setPower(power);
            }
        }
        else {
            spindexer.setPower(0);
        }
        return false;
    }




}