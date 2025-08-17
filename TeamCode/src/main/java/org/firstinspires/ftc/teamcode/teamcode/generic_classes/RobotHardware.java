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

-- Firstly make your class to be a subclass of a framework:
    public class hardvaretest extends OpModeFramework {
--Secondly init all the systems(or don`t if you want):
        public void runOpMode() throws InterruptedException {
        selfInit();               <-- only necessarily line of all initializations

        gyro.init_all();
        driveBase.init_all();
        servoMotors.init_all();

-- Then you can interact with motors:
        driveBase.FR = 1;             <-- sets motor power var in class
        FL.setPower(0);               <-- sets var in class trough motor class, will mostly work same with other classes
        driveBase.send_to_motors();   <-- sets powers to actual motors, will mostly work same with other classes
-Note: functions of type "send_to_motors" only accessible in "main thread"(that`s "public void runOpMode()")





IMPORTANT: THE SYSTEM IS "kinda" EXPERIMENTAL, so don`t implement it in main files yet
*/


















import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.HashMap;
import java.util.Map;
//experemental

public class RobotHardware{
    HardwareMap hardware;
    public RobotHardware(HardwareMap programmRobotHardwareMap){
        this.hardware = programmRobotHardwareMap;
    }
    public class GyroIMU{
        public BNO055IMU GyroIMU;
        private Orientation Orientation = new Orientation();
        private Acceleration Acceleration = new Acceleration();
        private boolean is_gyro_inited = false;
        public boolean is_gyro_enabled = false;

        public boolean is_inited(){
            return is_gyro_inited;
        }
        public void init_all(){
            init_all(true);
        }
        public void init_all(boolean do_enable){
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.mode = BNO055IMU.SensorMode.NDOF; // to usr accelerometer
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = true;

            GyroIMU = hardware.get(BNO055IMU.class, "imu");
            GyroIMU.initialize(parameters);

            is_gyro_inited = true;
            if(do_enable) {
                is_gyro_enabled = do_enable;
            }
        }

        public Orientation Angle() {
            if(!(is_gyro_inited&&is_gyro_enabled)){
                return Orientation;
            }
            Orientation = GyroIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            return Orientation;
        }

        public Acceleration Axel() {
            if(!(is_gyro_inited&&is_gyro_enabled)){
                Acceleration.xAccel = 0;
                Acceleration.yAccel = 0;
                Acceleration.zAccel = 0;
                return Acceleration;
            }
            Acceleration = GyroIMU.getLinearAcceleration();
            return Acceleration;
        }


    }

    public class DriveBase{
        public double FL=0, BL=0, FR=0, BR=0;
        public int FL_enc=0, BL_enc=0, FR_enc=0, BR_enc=0;
        public DcMotor hrd_FL, hrd_BL, hrd_FR, hrd_BR; // please avoid calling these

        private boolean is_drive_base_inited = false;
        public boolean is_drive_base_enabled = false;
        public boolean is_inited(){
            return is_drive_base_inited;
        }
        public void init_all(){
            init_all(true);
        }
        public void init_all(boolean do_enable){
            try {
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


                is_drive_base_inited = true;
            }
            catch (Exception ignored){}
            if(do_enable) {
                is_drive_base_enabled = do_enable;
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
        public void update_encoders(){
            FL_enc = hrd_FL.getCurrentPosition();
            FR_enc = hrd_FR.getCurrentPosition();
            BL_enc = hrd_BL.getCurrentPosition();
            BR_enc = hrd_BR.getCurrentPosition();
        }
        public void class_tick(){
            update_encoders();
            send_to_motors();
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
    public class Motors {
        private boolean is_motors_inited = false;
        public boolean is_servos_enabled = false;
        public boolean is_sparks_enabled = false;
        private Map<String,InternalServo> servos = new HashMap<>();
        private Map<String,InternalSpark> sparks = new HashMap<>();
        private Map<String,InternalSpark> motorsDC = new HashMap<>();

        public class servoKeys{
            // Class for easy renaming purposes
            public class servoNameKeys {
                public final static String apple_drop_module = "apple drop module";
                public final static String hidden_claw_module = "hidden claw";

                public class container_grab_module {
                    public final static String rightServo = "container_grab_R";
                    public final static String leftServo = "container_grab_L";
                }
            }
            public class SparkMiniNameKeys{
                public final static String extention_right = "extention_R";
                public final static String extention_left = "extention_L";
            }
            public class motorDCNameKeys {

            }
        }



        public Motors(){
            // Add motors HERE
            servos.put(servoKeys.servoNameKeys.apple_drop_module,new InternalServo("sbros"));
            servos.put(servoKeys.servoNameKeys.hidden_claw_module,new InternalServo("sbkr"));
            servos.put(servoKeys.servoNameKeys.container_grab_module.leftServo,new InternalServo("grabl"));
            servos.put(servoKeys.servoNameKeys.container_grab_module.rightServo,new InternalServo("grabr"));

//            sparks.put(servoKeys.SparkMiniKeys.extention_right,new InternalSpark("extr"));
//            sparks.put(servoKeys.SparkMiniKeys.extention_left,new InternalSpark("extl"));
        }


        public boolean is_inited(){
            return is_motors_inited;
        }
        public void init_all(){
            init_all(true,true);
        }
        public void init_all(boolean enable_servos){
            init_all(enable_servos,true);
        }
        public void init_all(boolean enable_servos, boolean enable_sparks){

            is_motors_inited = true;
            if (enable_servos) {
                is_servos_enabled = enable_servos;
            }
            if (enable_sparks){
                is_sparks_enabled = enable_sparks;
            }
        }



        public void send_to_components(){
            if(!is_motors_inited){
                return;
            }
            if(is_servos_enabled) {
                for (Map.Entry<String, InternalServo> servo_entry : servos.entrySet()) {
                    InternalServo servo_component = servo_entry.getValue();
                    if ((servo_component.AttachedComponent != null)&&(servo_component.is_powered)) {
                        servo_component.AttachedComponent.setPosition(servo_component.position);
                    }
                }
            }
            if(is_sparks_enabled) {
                for (Map.Entry<String, InternalSpark> servo_entry : sparks.entrySet()) {
                    InternalSpark spark_component = servo_entry.getValue();
                    if (spark_component.AttachedComponent != null) {
                        spark_component.AttachedComponent.setPower(spark_component.power);
                    }
                }
            }
        }



        private Servo getServoFunction(String name){
            Servo output;
            try {
                output = hardware.servo.get(name);
            }catch (Exception ignored){
                output = null;
            }
            return output;
        }



        private CRServo getSparkFunction(String name){
            CRServo output;
            try {
                output = hardware.crservo.get(name);
            }catch (Exception ignored){
                output = null;
            }
            return output;
        }



        private class InternalServo{
            public double position = 0;
            public boolean is_powered = false;
            public final Servo AttachedComponent;
            public InternalServo(String attachedComponentName){
                this.AttachedComponent = getServoFunction(attachedComponentName);
            }
        }



        private class InternalSpark{
            public double power = 0;
            public final CRServo AttachedComponent;
            public InternalSpark(String attachedComponentName){
                this.AttachedComponent = getSparkFunction(attachedComponentName);
            }
        }



        public class BasicServo{
            public final String component_Key;
            public final Motors parentClass;
            private final InternalServo attached_servo;
            public BasicServo(Motors parentClass, String Key){
                this.component_Key = Key;
                this.parentClass = parentClass;
                this.attached_servo = parentClass.servos.get(component_Key);
                if(attached_servo == null){
                    throw new IllegalArgumentException(String.format("RobotHardware.Servo(%s): Key error, no object found!!!",component_Key));
                }
            }
            public void setPosition(double pos){
                attached_servo.position = pos;
                attached_servo.is_powered = true;
            }
            public double getPosition(){
                return attached_servo.position;
            }
        }



        public class SparkMotor{
            public final String component_Key;
            public final Motors parentClass;
            private final InternalSpark attached_spark;
            public SparkMotor(Motors parentClass, String Key){
                this.component_Key = Key;
                this.parentClass = parentClass;
                this.attached_spark = parentClass.sparks.get(component_Key);
                if(attached_spark == null){
                    throw new IllegalArgumentException(String.format("RobotHardware.Spark(%s): Key error, no object found!!!",component_Key));
                }
            }
            public void setPower(double power){
                attached_spark.power = power;
            }
            public double getPower(){
                return attached_spark.power;
            }
        }

    }


}
