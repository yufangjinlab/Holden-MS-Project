package org.firstinspires.ftc.teamcode.Bot1.Hardware;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Objects;

public class Turret {
    HardwareMap hwMap = null;
    public CRServoImplEx turret;
    public Limelight3A limelight;
    Delivery delivery = new Delivery();
    public double BLUE_MAX_RIGHT = 0;
    public double BLUE_MAX_LEFT = 3;
    public double RED_MAX_RIGHT = -3;
    public double RED_MAX_LEFT = 0;

    public void init(HardwareMap hwMap){
        this.hwMap = hwMap;

        turret = hwMap.get(CRServoImplEx.class, "turret");
        limelight = hwMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(1);

        turret.setPower(0);

        turret.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void updateLimelightAngle(double distance, double xdistance, String color) {
        if (Objects.equals(color, "BLUE")) {
            if (
                    (distance < 45 && xdistance <= 108)
                    || (distance >= 45 && distance < 65 && xdistance <= 99)
                    || (distance >=65 && distance < 75 && xdistance <= 89)
                    || (distance >=75 && distance < 90 && xdistance <= 83)
                    || (distance >=90 && distance < 105 && xdistance <= 70)
                    || (distance >= 120)
            ) {
                BLUE_MAX_RIGHT = 0;
                BLUE_MAX_LEFT = 3;
            } else if (
                            (distance < 45 && xdistance > 108 && xdistance <= 110)
                            || (distance >= 45 && distance < 65 && xdistance > 99 && xdistance <= 103)
                            || (distance >=65 && distance < 75 && xdistance > 89 && xdistance <= 97)
                            || (distance >=75 && distance < 90 && xdistance > 83 && xdistance <= 94)
                            || (distance >=90 && distance < 105 && xdistance > 70 && xdistance <= 89)
            ) {
                BLUE_MAX_RIGHT = -2;
                BLUE_MAX_LEFT = 1;
            } else if (
                            (distance < 45 && xdistance > 110)
                            || (distance >= 45 && distance < 65 && xdistance > 103)
                            || (distance >=65 && distance < 75 && xdistance > 97)
                            || (distance >=75 && distance < 90 && xdistance > 94)
                            || (distance >=90 && distance < 105 && xdistance > 89)            ) {
                BLUE_MAX_RIGHT = -2.5;
                BLUE_MAX_LEFT = 0.5;
            } else {
                BLUE_MAX_RIGHT = -6.5;
                BLUE_MAX_LEFT = -3.5;
            }
        } else if (Objects.equals(color, "Red")) {
            if (
                    (distance < 45 && xdistance <= 108)
                            || (distance >= 45 && distance < 65 && xdistance <= 99)
                            || (distance >=65 && distance < 75 && xdistance <= 89)
                            || (distance >=75 && distance < 90 && xdistance <= 83)
                            || (distance >=90 && distance < 105 && xdistance <= 70)
                            || (distance >= 120)
            ) {
                RED_MAX_RIGHT = -3;
                RED_MAX_LEFT = 0;
            } else if (
                    (distance < 45 && xdistance > 108 && xdistance <= 110)
                            || (distance >= 45 && distance < 65 && xdistance > 99 && xdistance <= 103)
                            || (distance >=65 && distance < 75 && xdistance > 89 && xdistance <= 97)
                            || (distance >=75 && distance < 90 && xdistance > 83 && xdistance <= 94)
                            || (distance >=90 && distance < 105 && xdistance > 70 && xdistance <= 89)
            ) {
                RED_MAX_RIGHT = -1;
                RED_MAX_LEFT = 2;
            } else if (
                    (distance < 45 && xdistance > 110)
                            || (distance >= 45 && distance < 65 && xdistance > 103)
                            || (distance >=65 && distance < 75 && xdistance > 97)
                            || (distance >=75 && distance < 90 && xdistance > 94)
                            || (distance >=90 && distance < 105 && xdistance > 89)
            ) {
                RED_MAX_RIGHT = -0.5;
                RED_MAX_LEFT = 2.5;
            } else {
                RED_MAX_RIGHT = 3.5;
                RED_MAX_LEFT = 6.5;
            }
        }
    }
    public void aimTurret(Pose2D pose){
        double Heading = (-pose.getHeading(AngleUnit.RADIANS) * 180 / 3.14159265358979323) - 180;
        double DesiredHeadingRadians = Math.atan(delivery.YDisToGoal/delivery.XDisToGoal);
        double DesiredHeadingDegrees = (DesiredHeadingRadians * 180 / 3.14159265358979323) - 180;
    }
}
