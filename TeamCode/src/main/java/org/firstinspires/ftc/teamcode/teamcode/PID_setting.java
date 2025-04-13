package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PID_setting {
    public static double turnKp = 0.04, turnKd = 0.3, driveKp = 0.00025, driveKd = 0.0027,drivexKp = 0.00029, drivexKd = 0.0004;
    public static int targDist = 0;
    public static double  extpR = 0.004, extdR= 0,extpL = 0.004, extdL= 0, ignorex = 146, ignorey =155;

}
