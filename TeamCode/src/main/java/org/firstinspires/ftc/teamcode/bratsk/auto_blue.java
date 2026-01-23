package org.firstinspires.ftc.teamcode.bratsk;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;


import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.generic_classes.OpRobotSystemsFramework;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret.turet;
@Autonomous
public class auto_blue extends OpRobotSystemsFramework {
    public static double DISTANCE = 40;
    private boolean forward = true;

    private Path forwards;
    private Path backwards;
    public void idle_M(){
        spindexerModule.front_intaking = false;
        spindexerModule.front_shoot = false;
        spindexerModule.spin = 0;
        spindexerModule.enabled = true;
    }
    public void shoot_M(){
        spindexerModule.front_intaking = false;
        spindexerModule.front_shoot = true;
        spindexerModule.spin = 0;
        spindexerModule.enabled = true;
    }
    public void runOpMode() throws InterruptedException {
        selfInit();
        initAllSystems();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(120,0,Math.toRadians(180)));
        follower.update();
        waitForStart();
        follower.followPath(new Path(new BezierLine(new Pose(120,0), new Pose(96,48))));

        while(follower.isBusy()){
            follower.update();
            tickAll();
        }
        shoot_M();
        while(opModeIsActive()){
            tickAll();
        }

    }

}

