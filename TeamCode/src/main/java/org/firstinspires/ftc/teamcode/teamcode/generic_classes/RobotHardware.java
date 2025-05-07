package org.firstinspires.ftc.teamcode.teamcode.generic_classes;

/*

 /                  \
 |--Hardware Class--|
 \                  /

This is the new system for interacting with motors and sensors!
System includes simple implementation in "classic" programs,
It is solution to problem of each program having own initialization code identical to other programs.
No more copy-paste of initializations!

Instructions:

-- Firstly create objects:
    public void runOpMode() throws InterruptedException {
        RobotHardware robotHardware = new RobotHardware(hardwareMap);
        RobotHardware.DriveBase driveBase = robotHardware.new DriveBase();
        RobotHardware.DriveBase.motor_classes motor_classes = driveBase.new motor_classes();

        RobotHardware.DriveBase.motor_classes.FrontLeft FL = motor_classes.new FrontLeft(driveBase);
        RobotHardware.DriveBase.motor_classes.FrontRight FR = motor_classes.new FrontRight(driveBase);
        RobotHardware.DriveBase.motor_classes.BackLeft BL = motor_classes.new BackLeft(driveBase);
        RobotHardware.DriveBase.motor_classes.BackRight BR = motor_classes.new BackRight(driveBase);

-- Then you can interact with motors:
    driveBase.FR = 1;             <-- sets motor power var in class
    FL.setPower(0);               <-- sets var in class trough motor class
    driveBase.send_to_motors();   <-- sets powers to actual motors






IMPORTANT: THE SYSTEM IS EXPERIMENTAL, so don`t implement it in main files yet
*/

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//experemental

public class RobotHardware{
    HardwareMap hardware;
    public RobotHardware(HardwareMap programmRobotHardwareMap){
        this.hardware = programmRobotHardwareMap;
    }

    public class DriveBase{
        public double FL=0, BL=0, FR=0, BR=0;
        public int FL_enc=0, BL_enc=0, FR_enc=0, BR_enc=0;// FIXME: 07.05.2025 implement encoder reading from robot
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
            hrd_FL = hardware.dcMotor.get("FL");
            hrd_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hrd_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hrd_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            hrd_FR = hardware.dcMotor.get("FR");
            hrd_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hrd_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hrd_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            hrd_BL = hardware.dcMotor.get("BL");
            hrd_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hrd_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hrd_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            hrd_BR = hardware.dcMotor.get("BR");
            hrd_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hrd_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hrd_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.mode = BNO055IMU.SensorMode.NDOF;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = true;

            Gyro = hardware.get(BNO055IMU.class, "imu");
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

        public class motor_classes{
            public class FrontLeft{
                DriveBase driveBaseClassObject;
                public FrontLeft(DriveBase driveBaseClassObject){
                    this.driveBaseClassObject = driveBaseClassObject;
                }
                public void setPower(double power){
                    driveBaseClassObject.FL = power;
                }
                public double getPower(){
                    return driveBaseClassObject.FL;
                }
                public int getCurrentPosition(){
                    return driveBaseClassObject.FL_enc;
                }
            }
            public class FrontRight{
                DriveBase driveBaseClassObject;
                public FrontRight(DriveBase driveBaseClassObject){
                    this.driveBaseClassObject = driveBaseClassObject;
                }
                public void setPower(double power){
                    driveBaseClassObject.FR = power;
                }
                public double getPower(){
                    return driveBaseClassObject.FR;
                }
                public int getCurrentPosition(){
                    return driveBaseClassObject.FR_enc;
                }
            }
            public class BackLeft{
                DriveBase driveBaseClassObject;
                public BackLeft(DriveBase driveBaseClassObject){
                    this.driveBaseClassObject = driveBaseClassObject;
                }
                public void setPower(double power){
                    driveBaseClassObject.BL = power;
                }
                public double getPower(){
                    return driveBaseClassObject.BL;
                }
                public int getCurrentPosition(){
                    return driveBaseClassObject.BL_enc;
                }
            }
            public class BackRight{
                DriveBase driveBaseClassObject;
                public BackRight(DriveBase driveBaseClassObject){
                    this.driveBaseClassObject = driveBaseClassObject;
                }
                public void setPower(double power){
                    driveBaseClassObject.BR = power;
                }
                public double getPower(){
                    return driveBaseClassObject.BR;
                }
                public int getCurrentPosition(){
                    return driveBaseClassObject.BR_enc;
                }
            }
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


}
