package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servo_to_05 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo_0 = hardwareMap.servo.get("0");
        Servo servo_1 = hardwareMap.servo.get("1");
        Servo servo_2 = hardwareMap.servo.get("2");
        Servo servo_3 = hardwareMap.servo.get("3");
        Servo servo_4 = hardwareMap.servo.get("4");
        Servo servo_5 = hardwareMap.servo.get("5");
        Servo servo_6 = hardwareMap.servo.get("6");
        Servo servo_7 = hardwareMap.servo.get("7");
        Servo servo_8 = hardwareMap.servo.get("8");
        Servo servo_9 = hardwareMap.servo.get("9");
        Servo servo_10 = hardwareMap.servo.get("10");
        Servo servo_11 = hardwareMap.servo.get("11");
        waitForStart();
        while (opModeIsActive()) {
            servo_0.setPosition(0.5);
            servo_1.setPosition(0.5);
            servo_2.setPosition(0.5);
            servo_3.setPosition(0.5);
            servo_4.setPosition(0.5);
            servo_5.setPosition(0.5);
            servo_6.setPosition(0.5);
            servo_7.setPosition(0.5);
            servo_8.setPosition(0.5);
            servo_9.setPosition(0.5);
            servo_10.setPosition(0.5);
            servo_11.setPosition(0.5);
        }

    }
}
