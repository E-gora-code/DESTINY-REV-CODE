package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.generic_classes.OpModeFramework;

import java.util.List;
@TeleOp
public class configInstructions extends OpModeFramework {

    @Override
    public void runOpMode() throws InterruptedException {
        selfInit();
        initAllSystems();
        waitForStart();
        while (opModeIsActive()) {
            List<String> configServ = motors.getMotorsCorrectNames();
            for (String line : configServ) {
                printTelemetry("- ",line);
            }
            printTelemetry("","---------");
            List<String> configMiss = motors.getMotorsMissingNames();
            for (String line : configMiss) {
                printTelemetry("-! ",line);
            }
            dash.update();
            telemetry.update();
        }
    }
}
