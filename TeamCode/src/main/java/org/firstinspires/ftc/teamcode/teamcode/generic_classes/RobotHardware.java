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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.security.PublicKey;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;
//experemental

public class RobotHardware{
    public static List<Integer> checkErrorsIds = Arrays.asList();
    HardwareMap hardware;
    BiConsumer<Exception, Integer> errorHandler;

    public RobotHardware(HardwareMap programmRobotHardwareMap, BiConsumer<Exception, Integer> errorHandler){
        this.hardware = programmRobotHardwareMap;
        this.errorHandler = errorHandler;
    }
    public static class errorResponses{
        public static void raise(Exception e, int id){
            throw new IllegalArgumentException(e.getMessage());
        }
        public static void ignore(Exception e, int id){
        }
        public static void raise_id(Exception e, int id){

            if(checkErrorsIds.contains(id)){
                throw new IllegalArgumentException("!!ID: "+id+"\n"+e.getMessage());
            }
        }
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

        private DcMotor motor_init_helper(DcMotor motor,String name, int id){
            try {
                motor = hardware.dcMotor.get(name);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            catch (Exception ex){errorHandler.accept(ex,id);}
            return motor;
        }
        public void init_all(boolean do_enable){
            hrd_FL = motor_init_helper(hrd_FL,"FL",0);
            hrd_FR = motor_init_helper(hrd_FR,"FR",1);
            hrd_BL = motor_init_helper(hrd_BL,"BL",2);
            hrd_BR = motor_init_helper(hrd_BR,"BR",3);


            is_drive_base_inited = true;
            if(do_enable) {
                is_drive_base_enabled = do_enable;
            }
        }
        public void send_to_motors(){
            if(!(is_drive_base_inited&&is_drive_base_enabled)){
                return;
            }
            try {hrd_FL.setPower(_normolaize_DC(FL));}catch (Exception ex){errorHandler.accept(ex,4);}
            try {hrd_FR.setPower(_normolaize_DC(FR));}catch (Exception ex){errorHandler.accept(ex,5);}
            try {hrd_BL.setPower(_normolaize_DC(BL));}catch (Exception ex){errorHandler.accept(ex,6);}
            try {hrd_BR.setPower(_normolaize_DC(BR));}catch (Exception ex){errorHandler.accept(ex,7);}


        }
        public void update_encoders(){
            try {FL_enc = hrd_FL.getCurrentPosition();}catch (Exception ex){errorHandler.accept(ex,8);}
            try {FR_enc = hrd_FR.getCurrentPosition();}catch (Exception ex){errorHandler.accept(ex,9);}
            try {BL_enc = hrd_BL.getCurrentPosition();}catch (Exception ex){errorHandler.accept(ex,10);}
            try {BR_enc = hrd_BR.getCurrentPosition();}catch (Exception ex){errorHandler.accept(ex,11);}
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
        private boolean is_components_inited = false;
        public boolean is_servos_enabled = false;
        public boolean is_sparks_enabled = false;
        public boolean is_motors_enabled = false;
        private Map<String,InternalServo> servos = new HashMap<>();
        private Map<String,InternalSpark> sparks = new HashMap<>();
        private Map<String,InternalMotorDC> motorsDC = new HashMap<>();

        public class NameKeys {
            // Class for easy renaming purposes
            public class servoNameKeys {
                public final static String apple_drop_module = "apple drop module";
                public final static String hidden_claw_module = "hidden claw";

                public final static String factory_lift = "factory_lift";
                public class fork_module {
                    public final static String fork_right = "fork_right";
                    public final static String fork_left = "fork_left";
                }

                public class container_grab_module {
                    public final static String rightServo = "container_grab_R";
                    public final static String leftServo = "container_grab_L";
                }
            }
            public class SparkMiniNameKeys{
            }
            public class motorDCNameKeys {
                public final static String extention_right = "extention_R";
                public final static String extention_left = "extention_L";
                public final static String factory_extention = "factory_extention";
            }
        }



        public Motors(){
            // Add motors HERE
            servos.put(NameKeys.servoNameKeys.apple_drop_module,new InternalServo("sbros"));
            servos.put(NameKeys.servoNameKeys.hidden_claw_module,new InternalServo("sbkr"));
            servos.put(NameKeys.servoNameKeys.container_grab_module.leftServo,new InternalServo("grabl"));
            servos.put(NameKeys.servoNameKeys.container_grab_module.rightServo,new InternalServo("grabr"));

            servos.put(NameKeys.servoNameKeys.fork_module.fork_right,new InternalServo("vilar"));
            servos.put(NameKeys.servoNameKeys.fork_module.fork_left,new InternalServo("vilal"));

            servos.put(NameKeys.servoNameKeys.factory_lift,new InternalServo("reika",true, true));

            motorsDC.put(NameKeys.motorDCNameKeys.extention_right,new InternalMotorDC("extr"));
            motorsDC.put(NameKeys.motorDCNameKeys.extention_left,new InternalMotorDC("extl",-1));
            motorsDC.put(NameKeys.motorDCNameKeys.factory_extention,new InternalMotorDC("zavoz",-1));
        }


        public boolean is_inited(){
            return is_components_inited;
        }
        public void init_all(){
            init_all(true,true,true);
        }
        public void init_all(boolean enable_servos){
            init_all(enable_servos,true,true);
        }
        public void init_all(boolean enable_servos, boolean enable_sparks,boolean enable_DC){

            is_components_inited = true;
            if (enable_servos) {
                is_servos_enabled = enable_servos;
            }
            if (enable_sparks){
                is_sparks_enabled = enable_sparks;
            }
            if (enable_DC){
                is_motors_enabled = enable_DC;
            }
        }



        public void send_to_components(){
            if(!is_components_inited){
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
            if(is_motors_enabled) {
                for (Map.Entry<String, InternalMotorDC> motor_entry : motorsDC.entrySet()) {
                    InternalMotorDC motor_component = motor_entry.getValue();
                    if (motor_component.AttachedComponent != null) {
                        motor_component.AttachedComponent.setPower(motor_component.power);
                    }
                }
            }
        }
        public void update_encoders(){
        }
        public void class_tick(){
            update_encoders();
            send_to_components();
        }



        private Servo getServoFunction(String name){
            Servo output;
            try {
                output = hardware.servo.get(name);
            }catch (Exception ex){
                errorHandler.accept(ex,12);
                output = null;
            }
            return output;
        }



        private CRServo getSparkFunction(String name){
            CRServo output;
            try {
                output = hardware.crservo.get(name);
            }catch (Exception ex){
                errorHandler.accept(ex,13);
                output = null;
            }
            return output;
        }

        private DcMotor getMotorDCFunction(String name){
            DcMotor output;
            try {
                output = hardware.dcMotor.get(name);
            }catch (Exception ex){
                errorHandler.accept(ex,14);
                output = null;
            }
            return output;
        }

        private class InternalServo{
            public double position = 0;
            public boolean is_powered = false;
            public boolean is_continuous = false;
            public boolean is_reverse = false;
            public final Servo AttachedComponent;
            public InternalServo(String attachedComponentName){
                this.AttachedComponent = getServoFunction(attachedComponentName);
            }
            public InternalServo(String attachedComponentName,boolean is_reverse){
                this.AttachedComponent = getServoFunction(attachedComponentName);
                this.is_reverse = is_reverse;
            }
            public InternalServo(String attachedComponentName,boolean is_continuous, boolean is_reverse){
                this.AttachedComponent = getServoFunction(attachedComponentName);
                this.is_continuous = is_continuous;
                this.is_reverse = is_reverse;
            }
        }


        private class InternalSpark{
            public double power = 0;
            public final CRServo AttachedComponent;
            public InternalSpark(String attachedComponentName){
                this.AttachedComponent = getSparkFunction(attachedComponentName);
            }
        }

        private class InternalMotorDC{
            public double power = 0;
            public double encoder = 0;
            public double power_multiplier = 1;
            public final DcMotor AttachedComponent;
            public InternalMotorDC(String attachedComponentName){
                this.AttachedComponent = getMotorDCFunction(attachedComponentName);
            }
            public InternalMotorDC(String attachedComponentName,double power_mult){
                this.AttachedComponent = getMotorDCFunction(attachedComponentName);
                this.power_multiplier = power_mult;
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
            private double reversePosCheck(double pos){
                if (attached_servo.is_reverse) {
                    return (1 - pos);
                }
                else {
                    return pos;
                }
            }
            public void setPosition(double pos){
                attached_servo.position = reversePosCheck(pos);
                attached_servo.is_powered = true;
            }
            public double getCurrentPosition(){
                return reversePosCheck(attached_servo.position);
            }
            public void setPower(double power){
                attached_servo.position = reversePosCheck((power/2)+0.5);
                attached_servo.is_powered = true;
            }
            public double getPower(){
                return reversePosCheck((attached_servo.position-0.5)*2);
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

        public class DCMotor{
            public final String component_Key;
            public final Motors parentClass;
            private final InternalMotorDC attached_motorDC;
            public DCMotor(Motors parentClass, String Key){
                this.component_Key = Key;
                this.parentClass = parentClass;
                this.attached_motorDC = parentClass.motorsDC.get(component_Key);
                if(attached_motorDC == null){
                    throw new IllegalArgumentException(String.format("RobotHardware.DCMotor(%s): Key error, no object found!!!",component_Key));
                }
            }
            public void setPower(double power){
                attached_motorDC.power = power*attached_motorDC.power_multiplier;
            }
            public double getPower(){
                return attached_motorDC.power;
            }
        }

    }



    public class Sensors {
        private boolean is_components_inited = false;
        public boolean is_channels_enabled = false;

        private Map<String,InternalChannel> channels = new HashMap<>();

        public class NameKeys {
            // Class for easy renaming purposes
            public class channelsNameKeys {
                public final static String ch0 = "ch0";
                public final static String ch1 = "ch1";
            }
        }



        public Sensors(){
            // Add motors HERE
            channels.put(NameKeys.channelsNameKeys.ch0,new InternalChannel("ch0"));
            channels.put(NameKeys.channelsNameKeys.ch1,new InternalChannel("ch1"));
        }


        public boolean is_inited(){
            return is_components_inited;
        }
        public void init_all(){
            init_all(true);
        }
        public void init_all(boolean enable_channels){

            is_components_inited = true;
            if (enable_channels) {
                is_channels_enabled = enable_channels;
            }
        }



        public void read_from_components(){
            if(!is_components_inited){
                return;
            }
            if(is_channels_enabled) {
                for (Map.Entry<String, InternalChannel> channel_entry :  channels.entrySet()) {
                    InternalChannel channel_component = channel_entry.getValue();
                    if ((channel_component.AttachedComponent != null)) {
                        channel_component.state =channel_component.AttachedComponent.getState();
                    }
                }
            }
        }
        public void class_tick(){
            read_from_components();
        }



        private DigitalChannel getChannelFunction(String name){
            DigitalChannel output;
            try {
                output = hardware.digitalChannel.get(name);
            }catch (Exception ex){
                errorHandler.accept(ex,15);
                output = null;
            }
            return output;
        }



        private class InternalChannel{
            public boolean state = false;
            public boolean is_reverse = false;
            public final DigitalChannel AttachedComponent;
            public InternalChannel(String attachedComponentName){
                this.AttachedComponent = getChannelFunction(attachedComponentName);
            }
            public InternalChannel(String attachedComponentName,boolean is_reverse){
                this.AttachedComponent = getChannelFunction(attachedComponentName);
                this.is_reverse = is_reverse;
            }
        }




        public class BasicChannel{
            public final String component_Key;
            public final Sensors parentClass;
            private final InternalChannel attached_channel;
            public BasicChannel(Sensors parentClass, String Key){
                this.component_Key = Key;
                this.parentClass = parentClass;
                this.attached_channel = parentClass.channels.get(component_Key);
                if(attached_channel == null){
                    throw new IllegalArgumentException(String.format("RobotHardware.Servo(%s): Key error, no object found!!!",component_Key));
                }
            }
            private boolean reversePosCheck(boolean state){
                if (attached_channel.is_reverse) {
                    return !state;
                }
                else {
                    return state;
                }
            }
            public boolean getState(){
                return reversePosCheck(attached_channel.state);
            }
        }


    }


}
