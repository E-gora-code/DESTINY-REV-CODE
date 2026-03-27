package org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;


import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.generic_classes.OpModeFramework;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class auto extends OpMode {
    public static double DISTANCE = 30;
    private boolean forward = true;
    private spindexer spx;

    private Path forwards;
    private Path backwards;
    private turet turret;

    @Override
    public void init() {
        spx = new spindexer(hardwareMap);
        turret = new turet(hardwareMap, 5);
        follower = Constants.createFollower(hardwareMap);

        follower.startTeleopDrive();
        follower.update();

        follower.setStartingPose(new Pose(144,0,Math.toRadians(45)));
    }

    /** This initializes the Follower and creates the forward and backward Paths. */
    @Override
    public void init_loop() {

        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();
        forwards = new Path(new BezierLine(new Pose(144,0,Math.toRadians(45)), new Pose(-DISTANCE + 144,0+DISTANCE)));
        forwards.setConstantHeadingInterpolation(0);

        follower.followPath(forwards);
    }


    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
    @Override
    public void loop() {
        follower.update();
        draw();
        turret.update(follower.distansetogoalred(),-follower.angletogoalred(),follower.getVelocity().getXComponent(),-follower.getVelocity().getYComponent(),follower.getPose().getHeading(),gamepad2.right_stick_x);



        if (!follower.isBusy()){
            spx.update(false,false,false,true,0);
        }



    }
}