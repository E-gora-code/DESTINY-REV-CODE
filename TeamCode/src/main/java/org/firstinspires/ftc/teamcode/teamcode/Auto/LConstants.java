package org.firstinspires.ftc.teamcode.teamcode.Auto;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = 0.000568;
        TwoWheelConstants.strafeTicksToInches =0.000568;
        TwoWheelConstants.forwardY = -4.921;
        TwoWheelConstants.strafeX = -4.7;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "FL";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "FR";
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);


    }
}