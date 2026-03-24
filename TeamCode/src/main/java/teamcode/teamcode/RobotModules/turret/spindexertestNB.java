package teamcode.teamcode.RobotModules.turret;



import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.generic_classes.OpModeFramework;
import org.firstinspires.ftc.teamcode.pedro.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class spindexertestNB extends OpModeFramework {
    public static Follower follower;
    private spindexer spx;
    private turet turret;

    protected Telemetry dash = FtcDashboard.getInstance().getTelemetry();

    @Override
    public void runOpMode() {

        spx = new spindexer(hardwareMap);
        turret = new turet(hardwareMap, 5);
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        waitForStart();
        turret.reset();
        follower.setStartingPose(new Pose(0,0,Math.toRadians(180)));
        follower.update();
        follower.startTeleopDrive();
        follower.update();
        while (opModeIsActive()) {

            boolean intaking = gamepad1.right_bumper;
            boolean spining  = gamepad1.left_bumper;
            boolean shooting = gamepad1.a;
            if (gamepad2.a){
                follower.setPose(new Pose(144,144,Math.toRadians(-90)));
            }


            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
            follower.update();
            spx.update(false,gamepad1.right_bumper,false,"A",gamepad1.right_trigger>0.1,0);


            boolean shoot = gamepad1.right_trigger>0.1;
            double turn = gamepad2.left_stick_x;

            turret.update(follower.distansetogoalblue(),follower.angletogoalblue(),-follower.getVelocity().getXComponent(),-follower.getVelocity().getYComponent(),0);

            dash.addData("spx_pos", spx.v());
            dash.addData("spx_colG", spx.color_green());
            dash.addData("spx_colR", spx.color_blue());
            dash.addData("spx_colB", spx.color_red());
            dash.addData("spx_chekgreen", spx.chek_green_back());
            dash.addData("spx_chekpurple", spx.chek_purple_back());

            dash.addData("rpmR", turret.currRpmR());
            dash.addData("rpmErr", turret.rightpowe());
            dash.addData("targetRpm", turret.getTargetRpm());

            dash.addData("list",spx.bal());

            dash.update();
            telemetry.update();
        }

        turret.stop();
    }
}