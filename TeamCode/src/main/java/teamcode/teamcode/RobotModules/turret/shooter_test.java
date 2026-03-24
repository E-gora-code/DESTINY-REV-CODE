package teamcode.teamcode.RobotModules.turret;



import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.generic_classes.OpModeFramework;

@TeleOp(name = "Turret Test")
public class shooter_test extends OpModeFramework {
    private turet turret;

    protected Telemetry dash = FtcDashboard.getInstance().getTelemetry();


    @Override
    public void runOpMode() {



        turret = new turet(hardwareMap,5);


        waitForStart();
        turret.reset();

        while (opModeIsActive()) {
            boolean shoot = gamepad2.a;
            boolean droch = gamepad2.b;








            dash.addData("rightrpm",turret.currRpmR());
            dash.addData("rightrp",turret.rightpowe());




            dash.update();
            telemetry.update();

        }


    }
}