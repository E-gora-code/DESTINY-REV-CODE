package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PID_setting {
    public static double turnKp = 0.08, turnKd = 0.37, driveKp = 0.00018, driveKd = 0.003,drivexKp = 0.0002, drivexKd = 0.00048;
    public static int targDist = 0;
    public static double  extpR = 0.004, extdR= 0,extpL = 0.004, extdL= 0, ignorex = 146, ignorey =155;

}
