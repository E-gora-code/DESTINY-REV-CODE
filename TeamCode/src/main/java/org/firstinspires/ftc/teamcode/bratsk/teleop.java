package org.firstinspires.ftc.teamcode.bratsk;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.generic_classes.OpRobotSystemsFramework;
import org.firstinspires.ftc.teamcode.pedro.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret.spindexer;
import org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret.turet;
import org.firstinspires.ftc.teamcode.teamcode.control.actual.$Main_Control;
@TeleOp
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
        driveTrain.start();
        barsho.start();
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
                if(!gamepad1.ps) {
                    follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
                }else {
                    follower.setTeleOpDrive(-gamepad2.left_stick_y, -gamepad2.left_stick_x, -gamepad2.right_stick_x, false);
                }
                follower.update();
            }
            if (gamepad1.triangle){
                follower.setStartingPose(new Pose(10.6,9.5,Math.toRadians(0)));
                follower.startTeleopDrive();
                follower.update();
            }
        }

    }
    public class Barsho extends Thread{
        public void run() {
            waitForStart();
            while (opModeIsActive()) {

//                turret.update((gamepad2.right_trigger>0.2),gamepad2.left_stick_x);
                turret_x.setPower(controllerDriver_1.internal_touchpad.getX()+controllerDriver_2.internal_touchpad.getX());
                if(!gamepad1.ps) {
                    turret_y.setPosition((controllerDriver_1.internal_touchpad.getY() + gamepad2.left_stick_y) / 3);
                }
                if(gamepad1.back||gamepad2.back) {
                    spindexerModule.force_front = false;
                }
                spindexerModule.front_intaking = gamepad1.right_bumper||gamepad2.right_bumper;
                spindexerModule.front_shoot = gamepad1.left_bumper||gamepad2.left_bumper;
                spindexerModule.spin = gamepad2.right_stick_y*gamepad2.right_trigger;
                spindexerModule.enabled = true;
            }
    }
  }
  public boolean var_8_____D;
}

