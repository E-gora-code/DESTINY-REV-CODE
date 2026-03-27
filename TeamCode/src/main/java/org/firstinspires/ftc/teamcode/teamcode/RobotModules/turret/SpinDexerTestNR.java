package org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret;



import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;


import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.generic_classes.OpModeFramework;
import org.firstinspires.ftc.teamcode.pedro.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class SpinDexerTestNR extends OpModeFramework {
    public static Follower follower;
    private spindexer spx;
    private turet turret;
    double angle=0 , lastangle = 0;

    protected Telemetry dash = FtcDashboard.getInstance().getTelemetry();

    @Override
    public void runOpMode() {

        spx = new spindexer(hardwareMap);
        turret = new turet(hardwareMap, 5);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(8.18,144-7.7,Math.toRadians(0)));
        follower.startTeleopDrive();
        follower.update();
        waitForStart();
        while (opModeIsActive()) {

            boolean intaking = gamepad1.right_bumper;
            boolean spining  = gamepad1.left_bumper;
            boolean shooting = gamepad1.a;
            if (gamepad2.a){
                follower.setPose(new Pose(8.18,144-7.7,Math.toRadians(0)));
                turret_update();

            }
            angle = follower.getPose().getHeading();
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -0.3*gamepad1.right_stick_x, false);
            follower.update();
            spx.update(false,gamepad1.right_bumper,gamepad1.left_bumper,gamepad1.right_trigger>0.1,0);


            boolean shoot = gamepad1.right_trigger>0.1;
            double turn = gamepad2.left_stick_x;

            turret_update();

            dash.addData("spx_pos", spx.v());
            dash.addData("spx_colG", spx.color_green());
            dash.addData("spx_colB", spx.color_blue());
            dash.addData("spx_colR", spx.color_red());
            dash.addData("spx_chekgreen", spx.chek_green_back());
            dash.addData("spx_chekpurple", spx.chek_purple_back());

            dash.addData("rpmR", turret.get_current_turret_pose(false));
            dash.addData("rpmErr", turret.rightpowe());
            dash.addData("targetRpm", turret.getTargetRpm());
            dash.addData("angletogoal",Math.toDegrees(Math.PI/2-follower.angletogoalred()));
            dash.addData("distanse", follower.distansetogoalred());
            dash.addData("x",follower.getPose().getX());
            dash.addData("y",follower.getPose().getY());
            dash.addData("pos",turret.get_current_turret_pose(false));

            dash.addData("list",spx.bal());

            dash.update();
            telemetry.update();
            lastangle = angle;
        }

        turret.stop();
    }
    public void turret_update(){
        turret.update(follower.distansetogoalred()-5.5118,-follower.angletogoalred(),follower.getVelocity().getXComponent(),-follower.getVelocity().getYComponent(),angle,gamepad2.right_stick_x,gamepad2.right_stick_y);
    }
}