package org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret;



import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "sp")
public class spindexer_test extends OpMode {
    private SpindexerModule spindexe;
    protected Telemetry dash = FtcDashboard.getInstance().getTelemetry();
    DcMotorEx front_intake;




    @Override
    public void init() {
        front_intake = hardwareMap.get(DcMotorEx.class, "Fin");
        spindexe = new SpindexerModule(hardwareMap);

    }

    @Override
    public void loop() {
        front_intake.setPower(0);
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
        spindexe.update(true,gamepad1.right_bumper,false,gamepad1.right_trigger>0.1,pos);


        ;

    }
}
