package org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Spindexer Test", group = "Test")
public class spindexer_test extends LinearOpMode {

    private spindexer spindexerModule;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean shooting = false;
    private boolean ready = false;

    @Override
    public void runOpMode() {
        spindexerModule = new spindexer(hardwareMap);
        telemetry.addData("Position", "%.2f revs", spindexerModule.getSpindexerPosition());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a && !lastA) {
                shooting = !shooting;
            }
            if (gamepad1.b && !lastB) {
                ready = !ready;
            }
            lastA = gamepad1.a;
            lastB = gamepad1.b;


            double spindexerPower = 0;

            spindexerPower = gamepad1.right_trigger;

            spindexerModule.spindexer.setPosition(spindexerPower);


//            spindexerModule.update(shooting, ready);


            telemetry.addData("Shooting", shooting ? "YES" : "NO");
            telemetry.addData("Ready", ready ? "YES" : "NO");
            telemetry.addData("Spindexer Power", "%.2f", spindexerPower);
            telemetry.addData("Spindexer Position",  spindexerModule.getSpindexerPosition());
            telemetry.addData("Position Degrees",
                    (spindexerModule.getSpindexerPosition()) * 360);
            telemetry.update();

        }
    }
}