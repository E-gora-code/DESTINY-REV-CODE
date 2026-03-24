package org.firstinspires.ftc.teamcode.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.generic_classes.OpModeFramework;


@TeleOp
public class test extends OpModeFramework {



    @Override
    public void runOpMode() throws InterruptedException {
        selfInit();
        initAllSystems();
        waitForStart();
        while (opModeIsActive()) {
            back_ejector.setPosition(gamepad2.right_stick_x);
            tickAll();
        }
    }



}





