package org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.pedropathing.control.PIDFController;

public class turet {
    private Limelight3A limelight;
    private CRServo yaw;
    private Servo pitch;
    private  VoltageSensor voltageSensor;
    private PIDFController shooterLOutPid = new PIDFController(new PIDFCoefficients(0.0035, 0.0001, 0.0005, 0)),
            yawPid = new PIDFController(new PIDFCoefficients(0.0035, 0.0001, 0.0005, 0)),
            shooterROutPid = new PIDFController(new PIDFCoefficients(0.0035, 0.0001, 0.0005, 0));

    private AnalogInput shooterPos;
    private DigitalChannel shooter_zero ;
    public  double nominalVoltage = 12.0;
    public  double staticFrictionCoefficient = 0.1;
    private DcMotorEx shooterL, shooterR;

    private double filtTx = 0, filtTy = 0;
    private double point_of_potuga = 2;


    private double targetRpm = 240;
   

    private double voltage,shoter_zero=0;
    private double problem=0;
    private double lastVoltage = 0,revCount=0;
    private double power= 1;
    private double velocity = 100;



    public turet(HardwareMap hw,double pipeline) {
        voltageSensor = hw.voltageSensor.iterator().next();
        limelight = hw.get(Limelight3A.class, "limelight");
        yaw = hw.get(CRServo.class, "servoX");
        pitch = hw.get(Servo.class, "servoY");
        shooterL = hw.get(DcMotorEx.class, "shooterLeft");
        shooterR = hw.get(DcMotorEx.class, "shooterRight");
        shooterPos = hw.get(AnalogInput.class, "SP_POS");
        shooter_zero = hw.digitalChannel.get("0");
        shooterR.setDirection(DcMotorEx.Direction.REVERSE);
        shooterR.setDirection(DcMotorEx.Direction.REVERSE);
        shooterL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        yawPid.setCoefficients(new PIDFCoefficients(0.0035, 0.0001, 0.0005, 0));
        shooterLOutPid.setCoefficients(new PIDFCoefficients(0.0015, 0.0000001, 0.00006,0));
        shooterROutPid.setCoefficients(new PIDFCoefficients(0.0015, 0.0000001, 0.00006,0));
        limelight.pipelineSwitch(5);
    }

    public void reset() {
        pitch.setPosition(0.5);
        limelight.start();
    }

    public void update(boolean shoot,boolean droch) {
        yawPid.setCoefficients(new PIDFCoefficients(config.yawKp, config.yawKi, config.yawKd, 0));
        shooterLOutPid.setCoefficients(new PIDFCoefficients(config.shooterKp, config.shooterKi, config.shooterKd, 0));
        shooterROutPid.setCoefficients(new PIDFCoefficients(config.shooterKp*10/9, config.shooterKi, config.shooterKd, 0));

        LLResult r = limelight.getLatestResult();
        if (r != null && r.isValid()) {
            double tx = r.getTx();
            double ty = r.getTy();
            filtTx = 0.35 * tx + 0.65 * filtTx;
            filtTy = 0.35 * ty + 0.65 * filtTy;
            yawPid.updateError(filtTx);
            yawPid.run();

            double power = yawPid.run();


            yaw.setPower(clamp((power),-0.6,0.6));




            double yDist = get_distance();
            double discriminant = Math.pow(velocity,4) - 9.8  * (9.8 * yDist* yDist + 2 * 38.5* Math.pow(velocity,2));

            double sqrtDisc = Math.sqrt(discriminant);
            double tanTheta = (Math.pow(velocity,2) + sqrtDisc) / (9.8 * yDist);

            double angle =Math.atan(tanTheta);

            double pitchPos = angle*0.006875;
            pitchPos = clamp(pitchPos, 0.0, 1.0);
            pitch.setPosition(pitchPos);



            if (shoot) {
                double currRpmL = shooterL.getVelocity() * 60.0 / 560;
                shooterLOutPid.updateError(targetRpm-currRpmL);
                double shooterOutL = shooterLOutPid.run();
                double powerleft = (1.0/300) * targetRpm + shooterOutL;
                powerleft = clamp(powerleft, 0.0, 1.0);
                if (targetRpm <= 0) powerleft = 0;
                double currRpmR = -shooterR.getVelocity() * 60.0 / 560;
                shooterROutPid.updateError(targetRpm-currRpmR);
                double shooterOutR = shooterLOutPid.run();
                double powerright = (1.0/300) * targetRpm + shooterOutR;
                powerright = clamp(powerright, 0.0, 1.0);
                if (targetRpm <= 0) powerright = 0;
                voltage = voltageSensor.getVoltage();


                shooterL.setPower(powerleft*(nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient)));
                shooterR.setPower(powerright*(nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient)));
            }
            else{
                shooterL.setPower(0);
                shooterR.setPower(0);

            }
        }
        else{
            if (shoot&&droch){
                targetRpm = 240;
                double currRpmL = shooterL.getVelocity() * 60.0 / 560;
                shooterLOutPid.updateError(targetRpm-currRpmL);
                double shooterOutL = shooterLOutPid.run();
                double powerleft = (1.0/300) * targetRpm + shooterOutL;
                powerleft = clamp(powerleft, 0.0, 1.0);
                if (targetRpm <= 0) powerleft = 0;
                double currRpmR = shooterR.getVelocity() * 60.0 / 560;
                shooterROutPid.updateError(targetRpm-currRpmR);
                double shooterOutR = shooterLOutPid.run();
                double powerright = (1.0/270) * targetRpm + shooterOutR;
                powerright = clamp(powerright, 0.0, 1.0);
                if (targetRpm <= 0) powerright = 0;
                voltage = voltageSensor.getVoltage();

                shooterL.setPower(powerleft*(nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient)));
                shooterR.setPower(powerright*(nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient)));
                shooterR.setPower(-1);
                shooterR.setPower(1);

            }
            else{
                shooterL.setPower(0);
                shooterR.setPower(0);

            }

            yaw.setPower(clamp(power,-0.1,0.1));
            if ((get_current_turret_pose() - point_of_potuga) < 0.1){
                 power = clamp(power,1,1);
            }
            if ((get_current_turret_pose() + point_of_potuga) < 0.1){
                power = clamp(power,-1,0);
            }
        }
    }

    public boolean ready() {
        double currRpmL = shooterL.getVelocity() * 60.0 / 560;
        double ratioL = targetRpm > 0 ? currRpmL / targetRpm : 0;
        double currRpmR = shooterR.getVelocity() * 60.0 / 560;
        double ratioR = targetRpm > 0 ? currRpmR / targetRpm : 0;
        return targetRpm > 10 && ratioL > 0.95 && ratioR > 0.95 && Math.abs(filtTx) < 0.5;
    }
    public  double currRpmR(){
        double currRpmR = shooterR.getVelocity() * 60.0 / 560;
        return currRpmR;
    }
    public  double currRpmL(){
        double currRpmL = shooterL.getVelocity() * 60.0 / 560;
        return currRpmL;
    }


    public void stop() {
        yaw.setPower(0);
        shooterL.setPower(0);
        shooterR.setPower(0);
        limelight.stop();
    }
    public double get_current_turret_pose() {
        double v = shooterPos.getVoltage();
        if (lastVoltage > 3.0 && v < 0.3) revCount+=3;
        if (lastVoltage < 0.3 && v > 3.0) revCount-=3;
        lastVoltage = v;

        if (shooter_zero.getState()) {
            return revCount + (v)-shoter_zero;
        }
        else{
            revCount = 0;
            shoter_zero = v;
            return 0;
        }

    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
    public double getTargetRpm(){
        return targetRpm;
    }
    public double getFiltTx() {
        return filtTx;
    }
    public double getFiltTy() {
        return filtTy;
    }
    public double get_distance(){return limelight.getLatestResult().getTa();}
    public boolean sbros(){

        return shooter_zero.getState();
    }

    public double faund() {
        double sceal = 30000;
        double distancee = (sceal / limelight.getLatestResult().getTa());
        return distancee;

    }
}