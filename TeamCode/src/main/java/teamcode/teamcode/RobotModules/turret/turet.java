package teamcode.teamcode.RobotModules.turret;

import com.pedropathing.control.PIDFCoefficients;
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
    double tx = 0 ;
    private VoltageSensor voltageSensor;
    double pitchPos;
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
    private double angle;
    private double targetRpm = 240;
    private double voltage,shoter_zero=0;
    private double problem=0;
    private double lastVoltage = 0,revCount=0;
    private double power= 1,tanTheta;
    private double velocity = 100;
    double yDist = 0;

    private static final double GRAVITY = 386.4;
    private static final double LAUNCH_ANGLE = Math.toRadians(65);
    private static final double GOAL_HEIGHT = 24;

    public void setServoSpeed(Servo servo, double speed){
        servo.setPosition(speed/2+0.5);
    }

    public turet(HardwareMap hw,int pipeline) {
        voltageSensor = hw.voltageSensor.iterator().next();
        limelight = hw.get(Limelight3A.class, "limelight");
        yaw = hw.get(CRServo.class, "servoX");
        shooterL = hw.get(DcMotorEx.class, "shooterLeft");
        shooterR = hw.get(DcMotorEx.class, "shooterRight");
        shooterPos = hw.get(AnalogInput.class, "SP_PS");
        shooter_zero = hw.digitalChannel.get("0");
        shooterR.setDirection(DcMotorEx.Direction.REVERSE);
        shooterR.setDirection(DcMotorEx.Direction.REVERSE);
        shooterL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        yawPid.setCoefficients(new PIDFCoefficients(config.yawKp, config.yawKi, config.yawKd, 0));
        shooterLOutPid.setCoefficients(new PIDFCoefficients(config.shooterKp, config.shooterKi, 0, 1.0/250));
        shooterROutPid.setCoefficients(new PIDFCoefficients(config.shooterKp, config.shooterKi, 0, 1.0/250));
        limelight.pipelineSwitch(pipeline);
    }

    public void reset() {
        limelight.start();
    }

    public void update(double distance, double angleToGoal, double robotVelocityX, double robotVelocityY,double  turn) {



        yawPid.setCoefficients(new PIDFCoefficients(config.yawKp, config.yawKi, config.yawKd, 0));
        shooterLOutPid.setCoefficients(new PIDFCoefficients(config.shooterKp, config.shooterKi, 0, 1/250));
        shooterROutPid.setCoefficients(new PIDFCoefficients(config.shooterKp, config.shooterKi, 0, 0));
        ShooterResult sh = calculateShot(distance,angleToGoal,robotVelocityX,robotVelocityY);
        double speed = sh.velocity;
        double angle = Math.toDegrees(-angleToGoal-turn);
        double ofset =Math.toDegrees(sh.turretOffset);
        yawPid.updateError(-angle*0.023644+get_current_turret_pose(false));

        targetRpm =speed*1.21-60;
        double currRpmR = shooterR.getVelocity() * 60.0 / 560;
        shooterROutPid.updateError(targetRpm+currRpmR);
        double powerright =  targetRpm/260;
        if (Math.abs(targetRpm+currRpmR)>5){
            powerright =  shooterROutPid.run()+targetRpm/260;
        }
        powerright = clamp(powerright, 0.0, 1.0);
        if (targetRpm <= 0) powerright = 0;
        voltage = voltageSensor.getVoltage();
        shooterL.setPower(powerright*(nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient)));
        shooterR.setPower(powerright*(nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient)));

        yaw.setPower(yawPid.run());
    }

    public ShooterResult calculateShot(double distance, double angleToGoal, double robotVelocityX, double robotVelocityY) {
        double baseVelocity = Math.sqrt((GRAVITY * distance * distance) / (2 * Math.pow(Math.cos(LAUNCH_ANGLE), 2) * (distance * Math.tan(LAUNCH_ANGLE) - GOAL_HEIGHT)));
        double flightTime = distance / (baseVelocity * Math.cos(LAUNCH_ANGLE));
        double directionX = Math.cos(angleToGoal);
        double directionY = Math.sin(angleToGoal);
        double radialVelocity = robotVelocityX * directionX + robotVelocityY * directionY;
        double tangentialVelocity = -robotVelocityX * directionY + robotVelocityY * directionX;
        double horizontalVelocity = distance / flightTime;
        double compensatedHorizontalVelocity = horizontalVelocity + radialVelocity;
        double newHorizontalVelocity = Math.sqrt(compensatedHorizontalVelocity * compensatedHorizontalVelocity + tangentialVelocity * tangentialVelocity);
        double turretOffset = Math.atan2(tangentialVelocity, compensatedHorizontalVelocity);
        double newLaunchVelocity = newHorizontalVelocity / Math.cos(LAUNCH_ANGLE);
        return new ShooterResult(newLaunchVelocity, turretOffset);
    }

    public boolean ready() {
        double currRpmL = shooterL.getVelocity() * 60.0 / 560;
        double ratioL = targetRpm > 0 ? currRpmL / targetRpm : 0;
        double currRpmR = shooterR.getVelocity() * 60.0 / 560;
        double ratioR = targetRpm > 0 ? currRpmR / targetRpm : 0;
        return targetRpm > 10 && ratioL > 0.95 && ratioR > 0.95 && Math.abs(filtTx) < 0.5;
    }

    public  double currRpmR(){
        return -shooterR.getVelocity() * 60.0 / 560;
    }

    public  double currRpmL(){
        return shooterL.getVelocity() * 60.0 / 560;
    }

    public void stop() {
        shooterL.setPower(0);
        shooterR.setPower(0);
        limelight.stop();
    }

    public double get_current_turret_pose(boolean sbros) {
        double v = shooterPos.getVoltage();
        if (lastVoltage > 3.0 && v < 0.3) revCount+=3;
        if (lastVoltage < 0.3 && v > 3.0) revCount-=3;
        lastVoltage = v;
        if (sbros){
            revCount = 0;
           shoter_zero = v;
           return 0;
        }
        else{
            return revCount + (v)-shoter_zero;
        }
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public double rightpowe(){return targetRpm+shooterR.getVelocity() * 60.0 / 560;}
    public double getTargetRpm(){return targetRpm;}
    public double getFiltTx() {return tx;}
    public double getFiltTy() {return filtTy;}
    public double get_distance(){return limelight.getLatestResult().getTa();}
    public boolean sbros(){return shooter_zero.getState();}
    public double faund() {return 280 / limelight.getLatestResult().getTa();}
    public double dist (){return angle;}

    public static class ShooterResult {
        private final double velocity;
        private final double turretOffset;
        public ShooterResult(double velocity, double turretOffset) {
            this.velocity = velocity;
            this.turretOffset = turretOffset;
        }
        public double getVelocity() {return velocity;}
        public double getTurretOffset() {return turretOffset;}
        public double getTurretOffsetDegrees() {return Math.toDegrees(turretOffset);}
    }
}