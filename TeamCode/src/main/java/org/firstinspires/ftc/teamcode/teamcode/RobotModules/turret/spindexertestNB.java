package org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret;



import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.generic_classes.GamepadDriver;
import org.firstinspires.ftc.teamcode.pedro.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class spindexertestNB extends LinearOpMode {
    public static Follower follower;
    private SpindexerModule spx;
    private turet turret;
    public double angle= 0,lastangle = 0 ;

    protected Telemetry dash = FtcDashboard.getInstance().getTelemetry();
    public GamepadDriver gmDriver_1;
    @Override
    public void runOpMode() {
        gmDriver_1 = new GamepadDriver(gamepad1);

        spx = new SpindexerModule(hardwareMap);
        turret = new turet(hardwareMap, 5);
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        waitForStart();
        turret.reset();
        follower.setStartingPose(new Pose(144,144,Math.toRadians(180)));
        follower.update();
        follower.startTeleopDrive();
        follower.update();
        while (opModeIsActive()) {
            gmDriver_1.class_tick();
            boolean intaking = gamepad1.right_bumper;
            boolean spining  = gamepad1.left_bumper;
            boolean shooting = gamepad1.a;
            if (gamepad2.a){
                follower.setPose(new Pose(144,144,Math.toRadians(180)));
                turret_update();
            }
            angle = Math.signum(follower.getPose().getHeading())*(Math.toRadians(180) - Math.abs(follower.getPose().getHeading()));
            if (Math.abs(lastangle - angle)>2){
                double l = angle;
                angle = Math.signum(lastangle)*(Math.toRadians(180) - Math.abs(l)) + lastangle;
            }

            follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
            follower.update();
            spx.update(false,gamepad1.right_bumper,false,gamepad1.right_trigger>0.1,0);


            boolean shoot = gamepad1.right_trigger>0.1;
            double turn = gamepad2.left_stick_x;

            turret_update();

            dash.addData("spx_pos", spx.v());
            dash.addData("spx_colG", spx.color_green());
            dash.addData("spx_colR", spx.color_blue());
            dash.addData("spx_colB", spx.color_red());
            dash.addData("spx_chekgreen", spx.chek_green_back());
            dash.addData("spx_chekpurple", spx.chek_purple_back());

            dash.addData("rpmR", turret.currRpmR());
            dash.addData("rpmErr", turret.rightpowe());
            dash.addData("targetRpm", turret.getTargetRpm());

            dash.addData("rpmR", turret.get_current_turret_pose(false));
            dash.addData("rpmErr", turret.rightpowe());
            dash.addData("targetRpm", turret.getTargetRpm());
            dash.addData("angletogoal",Math.toDegrees(Math.PI/2-follower.angletogoalblue()));
            dash.addData("distanse", follower.distansetogoalblue());
            dash.addData("x",follower.getPose().getX());
            dash.addData("y",follower.getPose().getY());
            dash.addData("y",Math.toDegrees(follower.getPose().getHeading()));

            dash.addData("list",spx.bal());

            dash.addData("list",spx.bal());

            dash.update();
            telemetry.update();
            lastangle = angle;
        }

        turret.stop();
    }
    public void turret_update(){
        turret.update(follower.distansetogoalblue(), follower.angletogoalblue(), follower.getVelocity().getXComponent(),-follower.getVelocity().getYComponent(),0,gamepad2.right_stick_x+gmDriver_1.internal_touchpad.touchpad1_X, gamepad2.right_stick_y+gmDriver_1.internal_touchpad.touchpad1_Y);
        turret.setAutoMotorEnabled(gamepad1.back);
    }

}