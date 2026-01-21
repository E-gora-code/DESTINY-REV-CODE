package org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.generic_classes.RobotHardware;

@Config
public class config{
    public static double Front_ejector_shoot_position = 1,Front_ejector_noshoot_position =0;
    public static double Back_ejector_shoot_position = 0,Back_ejector_noshoot_position =1;
    public static double Back_wall_spin_position = 1,Back_wall_nospin_position =0;
    public static double Front_wall_spin_position = 0,Front_wall_nospin_position =1;
    public static double shootersp = 240;
    public static double shooterKp = 0.01;
    public static double shooterKi = 0.0000001;
    public static double shooterKd = 0.00012;
    public static double yawKp = 0.002;
    public static double yawKi = 0.0000001;
    public static double yawKd = 0.0001;

    public static double pos_front_ejector =0.5,pos_back_ejector =0.5,pos_back_wall =0.5,pos_front_wall =0.5, pos_spindexer =0.5;

}
