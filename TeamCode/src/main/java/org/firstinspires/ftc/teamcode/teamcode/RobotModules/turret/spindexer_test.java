package org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.generic_classes.OpModeFramework;

@TeleOp(name = "Spindexer Test", group = "Test")
public class spindexer_test extends OpModeFramework {
    private DataPackageInitSpindexer InitPackage;
    private spindexer spindexerModule;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean shooting = false;
    private boolean ready = false;

    @Override
    public void runOpMode() {
        selfInit();
        initAllSystems();
        InitPackage = new DataPackageInitSpindexer(hardwareMap);
        InitPackage.spindexer = spindexer;
        InitPackage.Front_wall = front_wall;
        InitPackage.Back_wall = back_wall;
        InitPackage.Front_ejector = front_ejector;
        InitPackage.Back_ejector = back_ejector;
        InitPackage.Front_intake = front_intake;
        InitPackage.Back_intake = back_intake;
        InitPackage.Shooter1 = shooter_right;
        InitPackage.Shooter2 = shooter_left;
        spindexerModule = new spindexer(InitPackage,hardwareMap);
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



           spindexerModule.update(gamepad1.right_bumper,gamepad1.left_bumper,gamepad1.x,false,gamepad1.right_stick_x);



            telemetry.addData("Shooting", shooting ? "YES" : "NO");
            telemetry.addData("Ready", ready ? "YES" : "NO");
            telemetry.addData("Spindexer Power", "%.2f", spindexerPower);
            telemetry.addData("Spindexer Position",  spindexerModule.getSpindexerPosition());
            telemetry.addData("Position Degrees",
                    (spindexerModule.getSpindexerPosition()) * 360);
            telemetry.update();
            tickAll();
        }
    }
}
