package org.firstinspires.ftc.teamcode.Bot1.Game.TeleOp;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TelePIDConfig {
    public static double kP = 10;//20;//14
    public static double kI = 0.06;//0.5; 2nd best: kI = .05
    public static double kD = 0.1;//35;//20 2nd best: kI = .01
    public static double kF = 12;//4; kF = 13 w/o other parameters (all = 0)

    public static int rpm;
    public static int ticks;
    /*close range rpm ~48in from goal =~ 2600 => (3,5) tiles, left front corner*/
    /*mid range rpm ~72in from goal =~3050 => right back corner on shooting tip area*/
    /* far range rpm 57x, 8.5-9y = 3800*/

    //private VoltageSensor battery = hardwareMap.get(VoltageSensor.class, "Control Hub");
    //private int TARGET_LAUNCH_VELOCITY = TelePIDConfig.ticks; //1800; LAUNCH VELOCITY = 1764 TICKS
    //private int MIN_LAUNCH_VELOCITY = TARGET_LAUNCH_VELOCITY + 1;//1775; //3770
    //private int MAX_LAUNCH_VELOCITY = TARGET_LAUNCH_VELOCITY +80;// MAX VELOCITY = 1820 TICKS //3815
    //private static final int CLOSE_LAUNCH_VELOCITY = TelePIDCon

    /* PID rpm and ticks transformation
        /**************
         * Encoder Ticks per second=RPM*PPR/60
         * PPR for 312: 537.6
         * 312*537.6/60== 2793.6
         * 300*537.6/60==2688
         *
         * 2800 ticks per second for 6000 rpm motor
         * each rpm: 6000/2800
         * 4000 rpm ~ = 1866.66666
         ***************************************************************/
}