package org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class spindexer {
    private Servo Front_ejector,Back_ejector,Back_wall,Front_wall;
    public Servo spindexer;
    private AnalogInput spindexerPos;
    private DcMotorEx Rignt_intake,Left_intake;
    private double lastVoltage = 0;
    private int revCount = 0;

    public spindexer(HardwareMap hw) {
//        Front_ejector = hw.get(Servo.class, "FE");
//        Back_ejector = hw.get(Servo.class, "BE");
//        Back_wall = hw.get(Servo.class, "BW");
//        Front_wall = hw.get(Servo.class, "FW");
        spindexer = hw.get(Servo.class, "SP");
        spindexerPos = hw.get(AnalogInput.class, "SP_POS");
//        Left_intake = hw.get(DcMotorEx.class, "LI");
//        Rignt_intake = hw.get(DcMotorEx.class, "RI");
//        Left_intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        Rignt_intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        Left_intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        Rignt_intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        lastVoltage = spindexerPos.getVoltage();
    }

    public double getSpindexerPosition() {
        double v = spindexerPos.getVoltage();

        return (spindexerPos.getVoltage()-0.172)*121.539;
    }

    public void update(boolean shoot,boolean ready) {
        if (ready){
            Front_ejector.setPosition(config.Front_ejector_shoot_position);
            Back_ejector.setPosition(config.Back_ejector_shoot_position);
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