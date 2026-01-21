package org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret;

import org.firstinspires.ftc.teamcode.generic_classes.RobotHardware;

public class spindexer {
    private RobotHardware.Motors.BasicServo front_ejector,back_ejector,back_wall,front_wall;
    public RobotHardware.Motors.BasicServo spindexer;
    private RobotHardware.Motors.DCMotor Front_intake,Back_intake, Shooter1, Shooter2;

    public spindexer(DataPackageInitSpindexer pack) {
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
    public void front_intaking(double mult1,double mult2){
        back_ejector.setPosition(0.75);
        back_wall.setPosition(0);
        front_ejector.setPosition(0.7);
        front_wall.setPosition(1);

        Shooter1.setPower(-mult2);
        Shooter2.setPower(mult2);
        Front_intake.setPower(mult1);
        Back_intake.setPower(mult1);
    }
}