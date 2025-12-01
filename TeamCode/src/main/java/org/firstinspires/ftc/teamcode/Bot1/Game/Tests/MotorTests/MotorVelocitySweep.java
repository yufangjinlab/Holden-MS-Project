package org.firstinspires.ftc.teamcode.Bot1.Game.Tests.MotorTests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Disabled

@TeleOp(name="Motor Velocity Sweep", group="Control")
public class MotorVelocitySweep extends LinearOpMode {
    private static final String TAG = "FTC";

    DcMotorEx motor;
    VoltageSensor battery;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "sMotor");
        battery = hardwareMap.voltageSensor.iterator().next();

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Example PIDF values – tune for your motor
        PIDFCoefficients pidf = new PIDFCoefficients(10.0, 1.0, 0.5, 10);
        motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        double startVel = 100;    // ticks/sec
        double endVel   = 1500;   // ticks/sec
        double rampTime = 10.0;   // seconds per ramp


        // ramp up
        double startTime = getRuntime();
        while (opModeIsActive() && getRuntime() - startTime <= rampTime) {
            double elapsed = getRuntime() - startTime;
            double fraction = elapsed / rampTime;
            double targetVel = startVel + (endVel - startVel) * fraction;

            motor.setVelocity(targetVel);

            telemetry.addData("Phase", "Ramp Up");
            telemetry.addData("Target Velocity", targetVel);
            telemetry.addData("Current Velocity", motor.getVelocity());
            telemetry.addData("Voltage (V)", battery.getVoltage());
            telemetry.addData("Current (A)", motor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();

            Log.i(TAG, "Phase=RampUp Target=" + targetVel
                    + " Vel=" + motor.getVelocity()
                    + " V=" + battery.getVoltage()
                    + " A=" + motor.getCurrent(CurrentUnit.MILLIAMPS));

        }

        // ramp down
        startTime = getRuntime();
        while (opModeIsActive() && getRuntime() - startTime <= rampTime) {
            double elapsed = getRuntime() - startTime;
            double fraction = elapsed / rampTime;
            double targetVel = endVel - (endVel - startVel) * fraction;

            motor.setVelocity(targetVel);

            telemetry.addData("Phase", "Ramp Down");
            telemetry.addData("Target Velocity", targetVel);
            telemetry.addData("Current Velocity", motor.getVelocity());
            telemetry.addData("Voltage (V)", battery.getVoltage());
            telemetry.addData("Current (A)",  motor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();

            Log.i(TAG, "Phase=RampUp Target=" + targetVel
                    + " Vel=" + motor.getVelocity()
                    + " V=" + battery.getVoltage()
                    + " A=" + motor.getCurrent(CurrentUnit.MILLIAMPS)
                    + " Error=" + ((targetVel-motor.getVelocity())/targetVel)*100);

        }

        // stop motor
        motor.setVelocity(0);
    }
}
