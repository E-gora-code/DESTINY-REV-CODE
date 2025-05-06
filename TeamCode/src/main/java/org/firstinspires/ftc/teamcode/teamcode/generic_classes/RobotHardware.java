package org.firstinspires.ftc.teamcode.teamcode.generic_classes;

/*

 /                  \
 |--Hardware Class--|
 \                  /

This is the new system for interacting with motors and sensors!
No more copy-paste of initializations!

Instructions:

-- Firstly create objects:
    RobotHardware robotHardvare = new RobotHardware();
    RobotHardware.DriveBase driveBase = robotHardvare.new DriveBase();

-- Then you can interact with motors:
    driveBase.FR = 1;             <-- sets motor power var in class
    driveBase.send_to_motors();   <-- sets powers to actual motors






IMPORTANT: THE SYSTEM IS EXPERIMENTAL, so don`t implement it in main files yet
*/

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//experemental

public class RobotHardware extends LinearOpMode {

    public class DriveBase{
        public double FL=0, BL=0, FR=0, BR=0;
        public DcMotor hrd_FL, hrd_BL, hrd_FR, hrd_BR; // please avoid calling these
        public BNO055IMU Gyro;// FIXME: 06.05.2025 move into own class
        private Orientation Orientation = new Orientation();
        private Acceleration Acceleration = new Acceleration();
        private boolean is_drive_base_inited = false;
        public boolean is_drive_base_enabled = false;
        public void init_all(){
            init_all(true);
        }
        public void init_all(boolean do_enable){
            hrd_FL = hardwareMap.dcMotor.get("FL");
            hrd_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hrd_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hrd_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            hrd_FR = hardwareMap.dcMotor.get("FR");
            hrd_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hrd_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hrd_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            hrd_BL = hardwareMap.dcMotor.get("BL");
            hrd_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hrd_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hrd_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            hrd_BR = hardwareMap.dcMotor.get("BR");
            hrd_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hrd_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hrd_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.mode = BNO055IMU.SensorMode.NDOF;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = true;

            Gyro = hardwareMap.get(BNO055IMU.class, "imu");
            Gyro.initialize(parameters);


            is_drive_base_inited = true;
            if(do_enable) {
                is_drive_base_enabled = is_drive_base_inited;
            }
        }
        public void send_to_motors(){
            if(!(is_drive_base_inited&&is_drive_base_enabled)){
                return;
            }
            hrd_FL.setPower(_normolaize_DC(FL));
            hrd_FR.setPower(_normolaize_DC(FR));
            hrd_BL.setPower(_normolaize_DC(BL));
            hrd_BR.setPower(_normolaize_DC(BR));


        }
        private double _normolaize_DC(double input_power){
            final double max = 1;
            if(input_power >= 0) {
                return Math.min(input_power,max);
            }
            else {
                return Math.max(input_power,-max);
            }
        }

    }


    @Override
    public final void runOpMode() throws InterruptedException {
        //leave blank
    }
}
