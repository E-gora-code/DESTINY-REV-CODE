package org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret;



import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.generic_classes.OpModeFramework;

@TeleOp(name = "sp")
public class spindexer_test extends OpMode {
    private spindexer spindexe;
    protected Telemetry dash = FtcDashboard.getInstance().getTelemetry();




    @Override
    public void init() {
        spindexe = new spindexer(hardwareMap);

    }

    @Override
    public void loop() {

        double pos = 1;
        if (gamepad1.a){
            pos = 2;
        }
        if (gamepad1.b){
            pos = 3;
        }
        pos = config.pos;
        dash.addData("1",spindexe.v());
        dash.addData("red",spindexe.color_red());
        dash.addData("blue",spindexe.color_blue());
        dash.addData("green",pos);
        dash.update();
        spindexe.update(true,false,false,"N",false, (float) pos);

    }
}
