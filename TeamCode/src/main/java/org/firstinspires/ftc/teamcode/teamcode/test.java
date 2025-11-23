package org.firstinspires.ftc.teamcode.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class test extends LinearOpMode {
    Servo Rotation,changeangle;



    @Override
    public void runOpMode() throws InterruptedException {
        Rotation =hardwareMap.servo.get("rotation");
        changeangle = hardwareMap.servo.get("changeangle");

        waitForStart();
        while (opModeIsActive()) {
            Rotation.setPosition(gamepad1.left_stick_x);
            changeangle.setPosition(gamepad1.right_stick_y);

        }
    }



}





