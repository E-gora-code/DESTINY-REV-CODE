package org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.pedro.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;

import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "autored", group = "Autonomous")
@Configurable
public class autored extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private long stateStartTime;
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);


        follower.setStartingPose(new Pose(72, 136, Math.toRadians(0)));
        follower.update();

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }
    @Override
    public void loop() {
        follower.update();
//        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }
    public int autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                follower.followPath(paths.Path1);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    stateStartTime = System.currentTimeMillis(); // preload действие
                    pathState = 2;
                }
                break;

            case 2:
                if (System.currentTimeMillis() - stateStartTime > 400) {
                    follower.followPath(paths.Path2);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    stateStartTime = System.currentTimeMillis(); // intake
                    pathState = 4;
                }
                break;

            case 4:
                if (System.currentTimeMillis() - stateStartTime > 300) {
                    follower.followPath(paths.Path3);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    stateStartTime = System.currentTimeMillis(); // deliver
                    pathState = 6;
                }
                break;

            case 6:
                if (System.currentTimeMillis() - stateStartTime > 400) {
                    follower.followPath(paths.Path4);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    stateStartTime = System.currentTimeMillis(); // intake 2
                    pathState = 8;
                }
                break;

            case 8:
                if (System.currentTimeMillis() - stateStartTime > 300) {
                    follower.followPath(paths.Path5);
                    pathState = 9;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    stateStartTime = System.currentTimeMillis(); // deliver 2
                    pathState = 10;
                }
                break;

            case 10:
                if (System.currentTimeMillis() - stateStartTime > 400) {
                    follower.followPath(paths.Path6);
                    pathState = 11;
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    pathState = -1; // завершено
                }
                break;
        }

        return pathState;
    }
    public static class Paths {

        public PathChain Path1, Path2, Path3, Path4, Path5, Path6;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(87.622, 133.876),
                            new Pose(86.705, 108.075)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(86.705, 108.075),
                            new Pose(126.978, 108.309)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(126.978, 108.309),
                            new Pose(86.720, 138.502)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .setReversed()
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(86.720, 138.502),
                            new Pose(87.084, 83.698)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(87.084, 83.698),
                            new Pose(126.953, 84.055)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(126.953, 84.055),
                            new Pose(87.051, 138.698)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .setReversed()
                    .build();
        }
    }
}
