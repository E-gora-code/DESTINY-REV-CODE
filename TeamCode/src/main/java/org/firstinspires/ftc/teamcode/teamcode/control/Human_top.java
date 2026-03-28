package org.firstinspires.ftc.teamcode.teamcode.control;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.generic_classes.OpModeFramework;
import org.firstinspires.ftc.teamcode.pedro.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret.SpindexerModule;
import org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret.turet;
@TeleOp
public class Human_top extends OpModeFramework {
    SpindexerModule spindexe;
    public static Follower follower;
    private turet turret;
    @Override
    public void runOpMode() throws InterruptedException {
        selfInit();
        initAllSystems();
        spindexe = new SpindexerModule(hardwareMap);
        turret = new turet(hardwareMap, 5);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(8.18,144-7.7,Math.toRadians(0)));
        follower.startTeleopDrive();
        follower.update();
        waitForStart();
        while (opModeIsActive()){
            tickAll();
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -0.3*gamepad1.right_stick_x, false);
            follower.update();



        }
    }
}
