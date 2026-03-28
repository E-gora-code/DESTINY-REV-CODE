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
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(8.18,144-7.7,Math.toRadians(0)));
        follower.startTeleopDrive();
        follower.update();
        waitForStart();
        while (opModeIsActive()){
            tickAll();
                        follower.update();
            if(gamepad1.left_bumper) {


                shooter_left.setPower(0.3);
                shooter_right.setPower(-0.3);
            }else if(gamepad1.right_bumper){
                turret_x.setPower(gamepad1.right_stick_x);
                shooter_left.setPower(gamepad1.right_stick_y);
                shooter_right.setPower(-gamepad1.right_stick_y);
            }
            double turna = gamepad1.right_stick_x;
            if(gamepad1.right_bumper){
                turna = 0;
            }
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -0.3*turna, false);


            spindexer.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
            spindexer1.setPower(gamepad1.left_trigger-gamepad1.right_trigger);


        }
    }
}
