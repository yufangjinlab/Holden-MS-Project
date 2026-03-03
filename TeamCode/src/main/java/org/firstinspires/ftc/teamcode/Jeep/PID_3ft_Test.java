package org.firstinspires.ftc.teamcode.Jeep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Config
//@Autonomous(name="PID_3ft_Test", group="Auton")
public class PID_3ft_Test extends LinearOpMode {

    DcMotorEx backLeft, backRight;

    // ====== TUNE THESE LIVE FROM DASHBOARD ======
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0005;

    public static int TARGET_TICKS = 610;  // 3 ft

    // PID state
    double integral = 0;
    double lastError = 0;

    @Override
    public void runOpMode() {

        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        waitForStart();

        movePID(TARGET_TICKS);   // forward
        sleep(1000);
        movePID(0);  // backward
    }

    private void movePID(int targetTicks) {

        integral = 0;
        lastError = 0;

        while (opModeIsActive()) {

            int currentPosition =
                    (backLeft.getCurrentPosition() +
                            backRight.getCurrentPosition()) / 2;

            double error = targetTicks - currentPosition;

            integral += error;
            double derivative = error - lastError;

            double output =
                    (kP * error) +
                            (kI * integral) +
                            (kD * derivative);

            // Clamp power
            output = Math.max(-1, Math.min(1, output));

            backLeft.setPower(output);
            backRight.setPower(output);

            telemetry.addData("Target", targetTicks);
            telemetry.addData("Position", currentPosition);
            telemetry.addData("Error", error);
            telemetry.addData("Output", output);
            telemetry.update();

            if (Math.abs(error) < 10) break;

            lastError = error;
        }

        backLeft.setPower(0);
        backRight.setPower(0);
    }
}