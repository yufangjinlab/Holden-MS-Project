package org.firstinspires.ftc.teamcode.Bot1.Hardware;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Bot1.Game.TeleOp.TelePIDConfig;

import java.util.Objects;

public class Delivery {
    HardwareMap hwMap = null;
    public DcMotorEx launcher;
    public CRServoImplEx rightFeeder, leftFeeder;
    public int TARGET_LAUNCH_VELOCITY, MIN_LAUNCH_VELOCITY, MAX_LAUNCH_VELOCITY;
    public static double xIniPosition;
    public static double yIniPosition;
    private static final double xBlueGoal = 137;
    private static final double yBlueGoal = 133;
    private static final double xRedGoal = 137;
    private static final double yRedGoal = 9;
    private static final double angleInDegrees = 75;
    private static final double angleInRadians = Math.toRadians(angleInDegrees);
    private static final double tangentValue = Math.tan(angleInRadians);
    private static final double cosineValue = Math.cos(angleInRadians);
    public double realX, realY;
    public double XDisToGoal, YDisToGoal;
    public double DistanceToGoal, DesiredLinearShootingVelocity, DesiredShootingVelocity;
    double efficiency;

    public void init(HardwareMap hwMap){
        this.hwMap = hwMap;

        this.launcher = hwMap.get(DcMotorEx.class, "launcher");
        this.rightFeeder = hwMap.get(CRServoImplEx.class, "rightFeeder");
        this.leftFeeder = hwMap.get(CRServoImplEx.class, "leftFeeder");

        this.launcher.setDirection(DcMotorEx.Direction.FORWARD);

        this.launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        this.launcher.setPower(0);

        this.rightFeeder.setPower(0);
        this.leftFeeder.setPower(0);

        this.rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE );

        PIDFCoefficients TelePIDF = new PIDFCoefficients(TelePIDConfig.kP, TelePIDConfig.kI, TelePIDConfig.kD, TelePIDConfig.kF);
        this.launcher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, TelePIDF);
    }
//    public void updateBlueRPM(Pose2D pose){
//        this.realX = xIniPosition + pose.getX(DistanceUnit.INCH);
//        this.realY = yIniPosition + pose.getY(DistanceUnit.INCH);
//        this.BlueXDisToGoal = realX - xBlueGoal;
//        this.BlueYDisToGoal = realY - yBlueGoal;
//        this.BlueDistanceToGoal= Math.sqrt((BlueXDisToGoal * BlueXDisToGoal) + (BlueYDisToGoal * BlueYDisToGoal)) + 4;
//        this.BlueDesiredLinearShootingVelocity = BlueDistanceToGoal * Math.sqrt((386.09) / (2 * cosineValue * cosineValue * (BlueDistanceToGoal * tangentValue - 26)));
//        if (BlueDistanceToGoal > 115){
//            efficiency = 0.45;
//        } else if (BlueDistanceToGoal < 65){
//            efficiency = 0.40;
//        } else if ((BlueDistanceToGoal >=65) && (BlueDistanceToGoal < 75)){
//            efficiency = 0.4225;
//        } else if ((BlueDistanceToGoal >= 75) && (BlueDistanceToGoal < 90)) {
//            efficiency = 0.425;
//        } else {
//            efficiency = 0.4275;
//        }
//        this.BlueDesiredShootingVelocity = (BlueDesiredLinearShootingVelocity*60)/(efficiency*3.14*3.75);
//        TelePIDConfig.rpm = (int) BlueDesiredShootingVelocity;
//        TelePIDConfig.ticks = (int) (BlueDesiredShootingVelocity * 28)/60;
//        this.TARGET_LAUNCH_VELOCITY = TelePIDConfig.ticks;
//        this.MIN_LAUNCH_VELOCITY = TARGET_LAUNCH_VELOCITY - 10;
//        this.MAX_LAUNCH_VELOCITY = TARGET_LAUNCH_VELOCITY + 40;
//    }

    public double updateDistance(Pose2D pose, String color) {
        realX = xIniPosition + pose.getX(DistanceUnit.INCH);
        realY = yIniPosition + pose.getY(DistanceUnit.INCH);
        if (Objects.equals(color, "BLUE")) {
            XDisToGoal = realX - xBlueGoal;
            YDisToGoal = realY - yBlueGoal;
        } else if (Objects.equals(color, "RED")) {
            XDisToGoal = realX - xRedGoal;
            YDisToGoal = realY - yRedGoal;
        }
        DistanceToGoal = Math.sqrt((XDisToGoal * XDisToGoal) + (YDisToGoal * YDisToGoal)) + 4;
        return DistanceToGoal;
    }

    public void updateRPM(double distance, String color){
        DesiredLinearShootingVelocity = distance * Math.sqrt((386.09) / (2 * cosineValue * cosineValue * (distance * tangentValue - 26)));
        if (distance < 45){
            efficiency = 0.38;
        } else if ((distance >= 45) && (distance < 65)){
            efficiency = 0.395;
        } else if ((distance >=65) && (distance < 75)){
            efficiency = 0.405;
        }  else if ((distance >= 75) && (distance < 90)) {
            efficiency = 0.414;
        } else if ((distance >= 90) && (distance < 105)) {
            efficiency = 0.42;
        } else if ((distance >= 105) && (distance < 120)){
            efficiency = 0.415;
        } else if ((distance >= 120) && (distance < 150)){
            efficiency = 0.44;
        } else {
            efficiency = 0.447;
        }
        if (Objects.equals(color, "RED")){
            if (distance > 80 && distance <= 120) {
                efficiency += 0.015;
            } else if (distance <  80){
                efficiency += 0.021;
            }
        }
            DesiredShootingVelocity = (DesiredLinearShootingVelocity*60)/(efficiency*3.14*3.75);
        TelePIDConfig.rpm = (int) DesiredShootingVelocity;
        TelePIDConfig.ticks = (int) (DesiredShootingVelocity * 28)/60;
        TARGET_LAUNCH_VELOCITY = TelePIDConfig.ticks;
        MIN_LAUNCH_VELOCITY = TARGET_LAUNCH_VELOCITY - 10;
        MAX_LAUNCH_VELOCITY = TARGET_LAUNCH_VELOCITY + 55;
    }
}
