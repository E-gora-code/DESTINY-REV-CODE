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
public class mmmm extends LinearOpMode {
    Telemetry dash = FtcDashboard.getInstance().getTelemetry();
    boolean is_mmmm = true;
    boolean is_color = true;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("r",gamepad1.right_stick_x);
            telemetry.addData("g",gamepad1.right_stick_y);
            telemetry.addData("b",gamepad1.left_stick_x);
            telemetry.update();
            dash.update();
            if(gamepad1.dpad_up||gamepad2.dpad_up){
                is_mmmm = false;
            }
            if (is_mmmm) {
                gamepad1.rumble(100);
                gamepad2.rumble(100);
            }
            if((gamepad1.right_stick_x+gamepad1.right_stick_y+gamepad1.left_stick_x+gamepad1.left_stick_y)==0) {
                gamepad1.setLedColor((Math.random()), (Math.random()), (Math.random()), 50);
                gamepad2.setLedColor((Math.random()), (Math.random()), (Math.random()), 50);
            }
            else{
                gamepad1.setLedColor(gamepad1.right_stick_x,gamepad1.right_stick_y,gamepad1.left_stick_x,100);
            }
        }


    }
}
