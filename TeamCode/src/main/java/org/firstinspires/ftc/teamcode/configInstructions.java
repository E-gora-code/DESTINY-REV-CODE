package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.generic_classes.OpModeFramework;
import org.firstinspires.ftc.teamcode.generic_classes.RobotHardware;

import java.util.List;

public class configInstructions extends OpModeFramework {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            List<String> configServ = motors.getServoConfigNames();
            for (String line : configServ) {
                printTelemetry("- ",line);
            }
            telemetry.update();
        }
    }
}
