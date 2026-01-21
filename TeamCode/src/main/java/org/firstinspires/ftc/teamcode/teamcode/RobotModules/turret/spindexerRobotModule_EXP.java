package org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret;

import org.firstinspires.ftc.teamcode.generic_classes.RobotHardware;

public class spindexerRobotModule_EXP {
    public double Front_ejector,Back_ejector,Back_wall,Front_wall;
    public double spindexer;
    private RobotHardware.Motors.DCMotor Front_intake,Back_intake;

    public spindexerRobotModule_EXP(DataPackageInitSpindexer pack) {

    }

    public void update(boolean shoot,boolean ready,boolean intaking) {
        if (intaking){
            Front_intake.setPower(1);
            Back_intake.setPower(1);
        }
        if (ready){
            Front_ejector = (config.Front_ejector_shoot_position);
            Back_ejector = (config.Back_ejector_shoot_position);
        }
        else{
            Front_ejector = (config.Front_ejector_noshoot_position);
            Back_ejector = (config.Back_ejector_noshoot_position);
        }
        if (shoot){
            Back_wall = (config.Back_wall_spin_position);
            Front_wall = (config.Front_wall_spin_position);
        }
        else{
            Back_wall = (config.Back_wall_nospin_position);
            Front_wall = (config.Front_wall_nospin_position);
        }
    }
}