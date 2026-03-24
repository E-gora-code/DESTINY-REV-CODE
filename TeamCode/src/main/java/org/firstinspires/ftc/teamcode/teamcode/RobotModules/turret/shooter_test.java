package org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret;



import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.generic_classes.OpModeFramework;

@TeleOp(name = "Turret Test")
public class shooter_test extends OpModeFramework {
    private turet turret;

    private DataPackageInitSpindexer InitPackage;
    private spindexer spindexerModule;


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

        turret = new turet(hardwareMap,5);


        waitForStart();
        turret.reset();

        while (opModeIsActive()) {
            boolean shoot = gamepad2.a;
            boolean droch = gamepad2.b;


            spindexerModule.front_intaking = gamepad2.dpad_down;
            spindexerModule.front_shoot = gamepad2.dpad_left;
            spindexerModule.spin = gamepad2.right_stick_y;
            spindexerModule.update_1ball();



            telemetry.addData("Target RPM", turret.getTargetRpm());
            telemetry.addData("Filtered Tx", turret.getFiltTx());
            telemetry.addData("Filtered Ty", turret.getFiltTy());
            telemetry.addData("Ready", turret.ready());
            telemetry.addData("Current Pose", turret.get_current_turret_pose());



            dash.addData("Target RPM", turret.getTargetRpm());
            dash.addData("Filtered Tx", turret.getFiltTx());
            dash.addData("Filtered Ty", turret.getFiltTy());
            dash.addData("Ready", turret.ready());
            dash.addData("Current Pose", turret.get_current_turret_pose());
            dash.addData("a", shoot);
            dash.addData("b", droch);
            dash.addData("leftrpm",turret.currRpmL());
            dash.addData("rightrpm",turret.currRpmR());
            dash.addData("distance",turret.faund());
            dash.addData("222",turret.sbros());
            dash.addData("dist",turret.dist());


            dash.update();
            telemetry.update();
            tickAll();
        }


    }
}