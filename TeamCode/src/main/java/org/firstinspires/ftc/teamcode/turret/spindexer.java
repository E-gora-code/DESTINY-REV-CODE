package org.firstinspires.ftc.teamcode.turret;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class spindexer {
    private Servo Front_ejector,Back_ejector,Back_wall,Front_wall;
    private CRServo spindexer;
    private DcMotorEx Rignt_intake,Left_intake;
    public spindexer(HardwareMap hw) {

        Front_ejector = hw.get(Servo.class, "FE");
        Back_ejector = hw.get(Servo.class, "BE");
        Back_wall = hw.get(Servo.class, "BW");
        Front_wall = hw.get(Servo.class, "FW");
        spindexer = hw.get(CRServo.class, "SP");
        Left_intake = hw.get(DcMotorEx.class, "LI");
        Rignt_intake = hw.get(DcMotorEx.class, "RI");
        Left_intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Rignt_intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Left_intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Rignt_intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void update(boolean shoot,boolean inttiangle) {
        if (shoot){

        }




    }

}