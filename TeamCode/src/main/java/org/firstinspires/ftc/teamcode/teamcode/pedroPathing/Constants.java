package org.firstinspires.ftc.teamcode.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teamcode.customconsts;
import org.firstinspires.ftc.teamcode.teamcode.pedro.Follower;
import org.firstinspires.ftc.teamcode.teamcode.pedro.FollowerBuilder;

// Убедитесь, что этот класс существует в вашем проекте


public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13)
            .forwardZeroPowerAcceleration(-41.9)
            .lateralZeroPowerAcceleration(-57.557)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.0005)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.03, 0.00, 0.02, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.1, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.008, 0, 0.0, 0, 0));

    public static customconsts driveConstants = new customconsts()
            .leftFrontMotorName("FL")
            .leftRearMotorName("BL")
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .MotorPower0(1)
            .MotorPower1(1)
            .MotorPower2(1)
            .MotorPower3(1)
            .MotorsScale(1)
            .xVelocity(98.8741)
            .yVelocity(82.7);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-6.9)
            .strafePodX(6.7)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            10,
            200,
            1,
            1.0
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .setDrivetrain(new CustomRrivetrain(hardwareMap, driveConstants))
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}