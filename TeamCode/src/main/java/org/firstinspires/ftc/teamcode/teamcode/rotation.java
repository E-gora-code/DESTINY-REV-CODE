package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.generic_classes.OpModeFramework;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class rotation extends OpModeFramework {
    ElapsedTime rpmTimer = new ElapsedTime();
    double rpm_zero = 0;
    double rpm;
    double m_power = 0.1;
    double m_power_l = 0;

    double target_rpm = -2000;
    double p = 1/1e4;
    double d = 1/1e3;



    List<Double> rpm_mean = new ArrayList<Double>(3);

    @Override
    public void runOpMode() throws InterruptedException {
        if(true){
            p = 1/1e6;
            d = 0;
        }
        selfInit();
        initAllSystems();
        waitForStart();
        rpmTimer.reset();
        rpm_zero = FR.getCurrentPosition();
        while (opModeIsActive()){
            FR.setPower(m_power);
            if(rpmTimer.seconds()>0.1){
                update_rpm();
                m_power = m_power+(target_rpm-rpm)*p+(m_power-m_power_l)*d;
                m_power_l = m_power;
            }
            tickAll();
        }

    }
    void update_rpm(){
        rpm_mean.add(((FR.getCurrentPosition()-rpm_zero)/rpmTimer.seconds()));
        if(rpm_mean.size()>3){
            rpm_mean.remove(0);
        }
        double mean_summ = 0;
        for(double i :rpm_mean){
           mean_summ += i;
        }
        rpm = mean_summ/rpm_mean.size();
        telemetry.addData("RPM", rpm);
        telemetry.addData("enc", FR.getCurrentPosition());
        telemetry.addData("power", FR.getPower());
        telemetry.update();
        rpm_zero = FR.getCurrentPosition();
        rpmTimer.reset();
    }
}
