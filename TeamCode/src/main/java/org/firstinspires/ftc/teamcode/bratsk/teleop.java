package org.firstinspires.ftc.teamcode.bratsk;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.generic_classes.OpRobotSystemsFramework;
import org.firstinspires.ftc.teamcode.pedro.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret.spindexer;
import org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret.turet;
import org.firstinspires.ftc.teamcode.teamcode.control.actual.$Main_Control;

public class teleop extends OpRobotSystemsFramework {
    public static Follower follower;
    Drive_train driveTrain = new Drive_train();
    Barsho barsho = new Barsho();
    private turet turret;

    @Override
    public void runOpMode() throws InterruptedException {
        selfInit();
        initAllSystems();
        follower = Constants.createFollower(hardwareMap);
        turret = new turet(hardwareMap,5);
        driveTrain.run();
        barsho.run();
        waitForStart();

        while(opModeIsActive()){

            tickAll();
        }

    }
    public class Drive_train extends Thread{
        public void run() {
            waitForStart();
            follower.setStartingPose(new Pose(10.6,9.5));
            follower.startTeleopDrive();
            follower.update();
            while (opModeIsActive()) {
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
                follower.update();
            }
        }

    }
    public class Barsho extends Thread{
        public void run() {
            waitForStart();
            while (opModeIsActive()) {
                turret.update((gamepad2.right_trigger>0.2),gamepad2.left_stick_x);

                spindexerModule.front_intaking = gamepad1.right_bumper;
                spindexerModule.front_shoot = gamepad1.right_trigger>0.2;
            }
    }
  }
  public boolean var_8_____D;
}

