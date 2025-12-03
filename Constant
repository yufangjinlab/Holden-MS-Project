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

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants().mass(10.8)
            .forwardZeroPowerAcceleration(-32.5)
            .lateralZeroPowerAcceleration(-60)
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.01, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015,0.0,0.0001,0.6,0.04))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.01));
    //mass must be in kilograms

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
           // .rightFrontMotorName("rightFront")
            //.rightRearMotorName("rightBack")
           //.leftRearMotorName("leftBack")
           // .leftFrontMotorName("leftFront")
            .rightFrontMotorName("leftBack")
            .rightRearMotorName("leftFront")
            .leftRearMotorName("rightFront")
            .leftFrontMotorName("rightBack")
            .leftFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD)
            .xVelocity(68)
            .yVelocity(50);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3.25)
            .strafePodX(6.375)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    /* YOU MAY NEED TO change yawScalar in PinpointCOnstants  */


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}

