package org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.generic_classes.RobotHardware;

public class spindexer {
    private RobotHardware.Motors.BasicServo front_ejector,back_ejector,back_wall,front_wall;
    public RobotHardware.Motors.BasicServo spindexer;
    private RobotHardware.Motors.DCMotor Front_intake,Back_intake, Shooter1, Shooter2;
    private RobotHardware.Sensors.BasicColorSensor sensor_ball;
    private boolean enabled_motors = false;
    public boolean enabled = false;

    public boolean front_intaking, back_intaking, back_shoot, front_shoot;
    public double spin , intake_speed = 0.3;
    private ElapsedTime last_intaked = new ElapsedTime();
    private ElapsedTime shooting_time = new ElapsedTime();
    private ElapsedTime color_time = new ElapsedTime();
    private boolean color_last = false;

    public int input_count = 0;

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
        this.sensor_ball = pack.colorSensor;


        last_intaked.reset();
        shooting_time.reset();
        color_time.reset();
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
    public boolean checkColor(){
        if(((sensor_ball.green()-32)>(sensor_ball.red()))){
            return true;
        }
        if(((sensor_ball.green()-10)>(sensor_ball.red()))&&((sensor_ball.blue()-5)>(sensor_ball.red()))){
            return true;
        }

        return false;
    }
    public void update_1ball(){
        if(this.enabled){this.enabled_motors = true;}
        if(!enabled_motors){return;}


         if (front_intaking){
             back_ejector.setPosition(0.75);
             back_wall.setPosition(0);
             front_ejector.setPosition(0.9);
             front_wall.setPosition(1);
             Front_intake.setPower(intake_speed);
             Back_intake.setPower(intake_speed);
             this.rotate_to(0.2,0.1);
             last_intaked.reset();
             shooting_time.reset();
         }
         else if (back_intaking){
             back_ejector.setPosition(0.75);
             back_wall.setPosition(0.8);
             front_ejector.setPosition(1);
             front_wall.setPosition(0);
             Front_intake.setPower(intake_speed);
             Back_intake.setPower(intake_speed);
             spindexer.setPower(spin);
             last_intaked.reset();
             shooting_time.reset();
         }
         else if (back_shoot){
             back_ejector.setPosition(1);
             Front_intake.setPower(1);
             Back_intake.setPower(1);
             spindexer.setPower(spin);
             shooting_time.reset();
         }
         else if (front_shoot){
             front_ejector.setPosition(0.7);
             Front_intake.setPower(0);
             Back_intake.setPower(0);
             if(shooting_time.seconds()<2) {
                 spindexer.setPower(0);
                 Shooter2.setPower(1);
                 Shooter1.setPower(-1);
             }else if(shooting_time.seconds()<3.5) {
                 Shooter2.setPower(1);
                 Shooter1.setPower(-1);
                 Front_intake.setPower(1);
                 Back_intake.setPower(1);
                 spindexer.setPower(-1);
             }
             else if(shooting_time.seconds()>6){
                 Front_intake.setPower(0);
                 Back_intake.setPower(0);
                 spindexer.setPower(0);
             }
         }
         else {
             Front_intake.setPower(0);
             Back_intake.setPower(0);
             if(spin==0) {
                 this.rotate_to(1.8, Math.max(1-last_intaked.seconds(),0)+0.1);
             }else {
                spindexer.setPower(spin);
             }
             shooting_time.reset();
             Shooter2.setPower(0);
             Shooter1.setPower(0);
         }


    }


    public void update_2ball(){
        if(this.enabled){this.enabled_motors = true;}
        if(!enabled_motors){return;}

        if (front_intaking){
            back_ejector.setPosition(0.65);
            back_wall.setPosition(0);
            front_ejector.setPosition(0.9);
            front_wall.setPosition(1);
            Front_intake.setPower(intake_speed);
            Back_intake.setPower(intake_speed);
            if(checkColor()){
                if(color_time.seconds()>1){
                    input_count = 1;
                }
            }else {
                color_time.reset();
            }
            if(input_count==0) {
                this.rotate_to(0.2, 0.1);
            }else {
                this.rotate_to(1.4, 0.1);
            }
            last_intaked.reset();
            shooting_time.reset();
        }

        else if (front_shoot){
            front_ejector.setPosition(0.7);
            Front_intake.setPower(0);
            Back_intake.setPower(0);
            if(shooting_time.seconds()<6) {
                back_ejector.setPosition(0.75);
                this.rotate_to(2, 0.5);
                Shooter2.setPower(1);
                Shooter1.setPower(-1);

            }else if(shooting_time.seconds()<4.5) {
                Shooter2.setPower(1);
                Shooter1.setPower(-1);
                Front_intake.setPower(1);
                Back_intake.setPower(1);
            }
            else if(shooting_time.seconds()<6.5) {
                back_ejector.setPosition(0.65);
                Shooter2.setPower(1);
                Shooter1.setPower(-1);
                Front_intake.setPower(1);
                Back_intake.setPower(1);
                spindexer.setPower(-1);
            }
            else if(shooting_time.seconds()>10){
                back_ejector.setPosition(0.65);
                Front_intake.setPower(0);
                Back_intake.setPower(0);
                spindexer.setPower(0);
            }
        }
        else {
            back_ejector.setPosition(0.65);
            Front_intake.setPower(0);
            Back_intake.setPower(0);
            if(spin==0) {
                this.rotate_to(1.8, Math.max(0.3-last_intaked.seconds(),0)+0.1);
            }else {
                spindexer.setPower(spin);
            }
            shooting_time.reset();
            Shooter2.setPower(0);
            Shooter1.setPower(0);
            input_count = 0;
            color_time.reset();
        }


    }
    public void update_3ball(){
        if(this.enabled){this.enabled_motors = true;}
        if(!enabled_motors){return;}

        if (front_intaking){
            back_ejector.setPosition(0.8);
            back_wall.setPosition(0);
            front_ejector.setPosition(0.9);
            front_wall.setPosition(1);
            Front_intake.setPower(intake_speed);
            Back_intake.setPower(intake_speed);
            if(checkColor()) {
                if (color_time.seconds() > 1) {
                    if((color_last!=checkColor())&&checkColor()) {
                        if (input_count < 2) {
                            input_count += 1;
                        }

                    }
                    color_last = checkColor();
                }
            }
            if (checkColor()){

            }else {
                color_time.reset();
                color_last = false;
            }

            if(input_count==0) {
                this.rotate_to(0.2, 0.1);
            }else if(input_count==1){
                this.rotate_to(1.4, 0.1);
            }else {
                Back_intake.setPower(0.3);
                this.rotate_to(2.6, 0.2);
            }
            last_intaked.reset();
            shooting_time.reset();
        }

        else if (front_shoot){
            front_ejector.setPosition(0.7);
            Front_intake.setPower(0);
            Back_intake.setPower(0);
            if(shooting_time.seconds()<6) {
                back_ejector.setPosition(0.75);
                this.rotate_to(2, 0.5);
                Shooter2.setPower(1);
                Shooter1.setPower(-1);

            }else if(shooting_time.seconds()<4.5) {
                Shooter2.setPower(1);
                Shooter1.setPower(-1);
                Front_intake.setPower(1);
                Back_intake.setPower(1);
            }
            else if(shooting_time.seconds()<6.5) {
                back_ejector.setPosition(0.65);
                Shooter2.setPower(1);
                Shooter1.setPower(-1);
                Front_intake.setPower(1);
                Back_intake.setPower(1);
                spindexer.setPower(-1);
            }
            else if(shooting_time.seconds()>10){
                back_ejector.setPosition(0.65);
                Front_intake.setPower(0);
                Back_intake.setPower(0);
                spindexer.setPower(0);
            }
        }
        else {
            Front_intake.setPower(0);
            Back_intake.setPower(0);
            if(spin==0) {
                this.rotate_to(1.8, Math.max(0.3-last_intaked.seconds(),0)+0.1);
            }else {
                spindexer.setPower(spin);
            }
            shooting_time.reset();
            Shooter2.setPower(0);
            Shooter1.setPower(0);
            input_count = 0;
            color_time.reset();
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