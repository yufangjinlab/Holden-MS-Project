package org.firstinspires.ftc.teamcode.Bot1.Game.Tests.MotorTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Config
@Disabled
public class PIDConfig {
    public static double kP =4.5;
    public static double kI = 1.0;
    public static double kD = 0.5;
    public static double kF = 4.0;

    public static double rpm=4000;

    public static double ticks = rpm/60*28;
}