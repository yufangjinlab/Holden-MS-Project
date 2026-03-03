package org.firstinspires.ftc.teamcode.Jeep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name="PID_ExternalEncoder_Loop", group="Auton")
public class PID_ExternalEncoder_Loop extends LinearOpMode {

    // Drive motors
    DcMotorEx backLeft, backRight;

    // ===== DASHBOARD TUNING =====
    public static double kP = 0.0003;
    public static double kI = 0.0;
    public static double kD = 0.00002;

    public static int TARGET_TICKS = 12120;  // 3 feet
    public static int TOLERANCE = 50;

    public static double MAX_POWER = 0.8;

    double integral = 0;
    double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        // If encoder counts backward, flip this
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset external encoder (plugged into backRight port)
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        waitForStart();

        int target = TARGET_TICKS;
        timer.reset();

        while (opModeIsActive()) {

            double dt = timer.seconds();
            timer.reset();

            int currentPosition = -backRight.getCurrentPosition();
            double error = target - currentPosition;

            // Integral with anti-windup clamp
            integral += error * dt;
            integral = clamp(integral, -5000, 5000);

            double derivative = (error - lastError) / dt;

            double output =
                    (kP * error) +
                            (kI * integral) +
                            (kD * derivative);

            output = clamp(output, -MAX_POWER, MAX_POWER);

            backLeft.setPower(output);
            backRight.setPower(output);

            telemetry.addData("Target", target);
            telemetry.addData("Position", currentPosition);
            telemetry.addData("Error", error);
            telemetry.addData("Output", output);
            telemetry.addData("dt", dt);
            telemetry.update();

            // Switch between 0 and 3ft
            if (Math.abs(error) < TOLERANCE) {

                integral = 0;
                lastError = 0;

                if (target == TARGET_TICKS) {
                    target = 0;
                } else {
                    target = TARGET_TICKS;
                }
            }

            lastError = error;
        }
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}