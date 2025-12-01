package org.firstinspires.ftc.teamcode.Bot1.Game.Tests.MotorTests;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Disabled

@TeleOp(name="Motor Velocity PID1", group="Control")
public class MotorVelocityConstant extends LinearOpMode {

    DcMotorEx motor;
    VoltageSensor battery;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        motor = hardwareMap.get(DcMotorEx.class, "sMotor");

        // Get first voltage sensor from the Control Hub
        battery = hardwareMap.voltageSensor.iterator().next();

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Example PIDF values (adjust for your motor)
        //PIDFCoefficients pidf = new PIDFCoefficients(15, 1, 0.5, 0);
        //motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);

        // Combine DS telemetry with FTC Dashboard telemetry

//        double kD = 0.5;
//        double kF = 0;
//        double kI = 1;
//        double kP = 15;
//        double rpm = 4000;
        // double ticks = 18670;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();


        /**************
         * Encoder Ticks per second=RPM*PPR/60
         * PPR for 312: 537.6
         * 312*537.6/60== 2793.6
         * 300*537.6/60==2688
         *
         * 2800 ticks per second for 6000 rpm motor
         * each rpm: 6000/2800
         */

        while (opModeIsActive()) {

            motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(PIDConfig.kP, PIDConfig.kI, PIDConfig.kD, PIDConfig.kF));

            double targetVel = PIDConfig.ticks;

            // Command motor velocity
            motor.setVelocity(targetVel);

            // Collect sensor data
            double velocity = motor.getVelocity();
            double current = motor.getCurrent(CurrentUnit.MILLIAMPS);
            double voltage  = battery.getVoltage();



            // Log to FTC Dashboard & DS telemetry
            telemetry.addData("Target Velocity", targetVel);
            telemetry.addData("Current Velocity", velocity*60/28); // in rpm
            telemetry.addData("Current (A)", current);
            telemetry.addData("Voltage (V)", voltage);
            telemetry.addData("P Coefficient", PIDConfig.kP);
            telemetry.addData("I Coefficient", PIDConfig.kI);
            telemetry.addData("D Coefficient", PIDConfig.kD);
            telemetry.addData("F Coefficient", PIDConfig.kF);
            telemetry.update();

            // Logcat (optional)
            Log.i("FTC", "Phase=const Target=" + targetVel
                    + " Vel=" + motor.getVelocity()
                    + " V=" + battery.getVoltage()
                    + " A=" + motor.getCurrent(CurrentUnit.MILLIAMPS)
                    + " Error=" + ((targetVel-velocity)/PIDConfig.ticks)*100);
            /*
            Log.i("FTC", "Phase=constant Target=" + targetVel +
                    " Vel=" + velocity +
                    " A=" + current +
                    " V=" + voltage);

             */
        }
    }
}