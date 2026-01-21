package org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret;



import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Turret Test")
public class shooter_test extends LinearOpMode {
    private turet turret;
    Telemetry dash = FtcDashboard.getInstance().getTelemetry();


    @Override
    public void runOpMode() {

        turret = new turet(hardwareMap,5);


        waitForStart();

        while (opModeIsActive()) {
            boolean shoot = gamepad1.a;
            boolean droch = gamepad1.b;

            turret.update(shoot, droch);

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
            dash.addData("distance",turret.get_distance());
            dash.addData("222",turret.sbros());

            dash.update();
            telemetry.update();
        }


    }
}