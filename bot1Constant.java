package org.firstinspires.ftc.teamcode.Pedro.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Bot1Constants {
    public static FollowerConstants followerConstants = new FollowerConstants().mass(10.8)
            .forwardZeroPowerAcceleration(-30.5)
            .lateralZeroPowerAcceleration(-59)

            .useSecondaryTranslationalPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.009,0.023))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.001,0,0.02,0.015))

            .useSecondaryHeadingPIDF(false)
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,0.0005,0.025))//1, 0, 0.0005, 0.025
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.25,0,0.02,0.01)) //0.25, 0, 0.02, 0.01

            .useSecondaryDrivePIDF(false)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.3,0,0.00001,0.6,0.047))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.005,0,0.000001,0.6,0.01));
    //mass must be in kilograms

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.9, 0.85); // 0.9, 0.85


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("leftBack")
            .rightRearMotorName("leftFront")
            .leftRearMotorName("rightFront")
            .leftFrontMotorName("rightBack")
            .leftFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD)
            .xVelocity(84.5)
            .yVelocity(65);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-4.1)
            .strafePodX(6.7)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
