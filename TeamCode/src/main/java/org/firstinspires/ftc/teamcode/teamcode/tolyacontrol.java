package org.firstinspires.ftc.teamcode.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp
public class tolyacontrol extends LinearOpMode {
    boolean xkfi = false;
    PID_setting pid_setting = new PID_setting();
    // config
    boolean simple_ext = true;
    double sbkrpos ;
    double powery =0 ;
    double powerx=0;
    // end
    double mult = 1;
    double extr_zero = 0, extl_zero = 0;
    double extr_max = 5170, extl_max;
    DcMotor FL, BL, FR, BR;
    Servo sbkr, grabr, grabl,sbros;
    CRServo extl,extr;
    double currentAngle = 0;
    double exterR,exterL,extpowerR,extpowerL,extrlR,extrlL;
    double extr_pos, extl_pos;


    double Multiply = 0.2;
    double turnPower = 0;
    double forwardPower = 0;
    double sidePower = 0;

    double turnDrift = 0;
    double forwardDrift = 0;
    double sideDrift = 0;

    double targAngle = 0;
    ElapsedTime driftCore = new ElapsedTime();

    double turnErr = 0, turnErrL = 0;
    double deltaHed = 0, deltaHedL = 0;
    double ext_pos_calk = 0;
    double gamepad_summ = 0;
    boolean lastext = false;
    boolean presed_reset_ang = false;
    boolean ext_press=false;
    boolean is_homed_ever = false;
    boolean b_press = false;
    boolean grab_toggle = false;
    boolean sbkr_toggle = false;
    boolean sbkr_press = false;
    ElapsedTime sbkr_need_reset = new ElapsedTime();
    ElapsedTime homatimr = new ElapsedTime();
    ElapsedTime lastctrl = new ElapsedTime();
    ElapsedTime hold_to_wake_homa = new ElapsedTime();
    ElapsedTime ext_timer = new ElapsedTime();
    ElapsedTime presed_reset_ang_timer = new ElapsedTime();
    ElapsedTime presed_reset_extencion_timer = new ElapsedTime();
    ElapsedTime continious_timer = new ElapsedTime();
    Driving driving = new Driving();
    boolean lastsbkr = false;

    double x = 0;;
    int click = 0;
    int clmult = 1;

    double pos_last = 0;
    double gm2ls_summ = 0;


    ElapsedTime extention_time = new ElapsedTime();
    double extention_time_interval = 10;

    double color_pulse = 1;
    ElapsedTime color_pulse_timer = new ElapsedTime();
    ElapsedTime drive = new ElapsedTime();



    boolean can_homa_wake_up = true;

    boolean ishoma = false;
    ElapsedTime lastcl = new ElapsedTime();
    boolean iscl1 = false;
    boolean iscl = false;
    boolean ves = false;
    BNO055IMU Gyro;
    Orientation Orientation = new Orientation();
    DigitalChannel ch0, ch1;

    Telemetry dash = FtcDashboard.getInstance().getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        FL = hardwareMap.dcMotor.get("FL");
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FR = hardwareMap.dcMotor.get("FR");
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BL = hardwareMap.dcMotor.get("BL");
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BR = hardwareMap.dcMotor.get("BR");
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ch0 = hardwareMap.digitalChannel.get("0");
        ch1 = hardwareMap.digitalChannel.get("1");
        ch0.setMode(DigitalChannel.Mode.INPUT);
        ch1.setMode(DigitalChannel.Mode.INPUT);




        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        Gyro = hardwareMap.get(BNO055IMU.class, "imu");


        Gyro.initialize(parameters);


        waitForStart();






        color_pulse_timer.reset();
        extention_time.reset();
        continious_timer.reset();
        sbkr_need_reset.reset();
        targAngle = Angle();
        driftCore.reset();
        driving.start();
        while (opModeIsActive()) {


        }

    }

    public double Angle() {
        Orientation = Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Orientation.firstAngle;
    }




    class Driving extends Thread {
        public void run() {
            while (opModeIsActive()) {
                if (gamepad1.left_trigger > 0.4){
                    Multiply = gamepad1.left_trigger *2;
                }
                else {
                    Multiply = 0.4;
                }

                if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0) {
                    mult = 1;
                    drive.reset();
                } else {
                    mult = Math.min(drive.milliseconds() * 0.04, 1);
                }


                double Angle = Angle();

                deltaHed = Angle - deltaHedL;

                if (deltaHed > 180) {
                    deltaHed -= 360;
                } else if (deltaHed < -180) {
                    deltaHed += 360;
                }
                currentAngle += deltaHed;

                if (gamepad1.right_stick_x == 0) {
                    turnErr = targAngle - currentAngle;
                    turnPower = turnErr * pid_setting.turnKp + (turnErr - turnErrL) * pid_setting.turnKd;
                    turnErrL = turnErr;
                } else {
                    targAngle = currentAngle;
                    turnPower = -gamepad1.right_stick_x * 0.8;
                }
                powery = Math.cos(Math.toRadians(currentAngle))*gamepad1.left_stick_x -  Math.sin(Math.toRadians(currentAngle)) * gamepad1.left_stick_y;
                powerx = Math.sin(Math.toRadians(currentAngle))*gamepad1.left_stick_x + Math.cos(Math.toRadians(currentAngle)) *  gamepad1.left_stick_y;
                FR.setPower(((-powerx  + powery + turnPower)) * (-Multiply * mult));
                FL.setPower(((powerx  + powery + turnPower)) * (-Multiply * mult));
                BR.setPower(((-powerx  - powery + turnPower)) * (-Multiply * mult));
                BL.setPower(((powerx  - powery + turnPower)) * (-Multiply * mult));
                dash.addData("positionx", BR.getCurrentPosition());
                dash.addData("positiony", BL.getCurrentPosition());
                dash.addData("ignorex", currentAngle);
                telemetry.addData("drive.milliseconds();", drive.milliseconds());
                telemetry.update();

                deltaHedL = Angle;
            }

        }
        public double SmartMin ( double a, double b){

            if (Math.abs(a) <= Math.abs(b)) {
                return a;
            } else {
                if (a >= 0) {
                    return b;
                } else {
                    return -b;
                }
            }
        }
    }
}





