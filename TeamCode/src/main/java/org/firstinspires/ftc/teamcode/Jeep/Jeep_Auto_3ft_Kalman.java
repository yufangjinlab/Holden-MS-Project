package org.firstinspires.ftc.teamcode.Jeep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Kalman.KalmanFilter;

@Autonomous(name = "Jeep: 3ft Forward/Back Kalman", group = "Jeep")
public class Jeep_Auto_3ft_Kalman extends LinearOpMode {

    private DcMotorEx backLeft, backRight;
    private DcMotor frontTurn;

    // ======== CONSTANTS ========
    private static final double TICKS_PER_REV = 2000;
    private static final double WHEEL_DIAMETER_INCHES = 1.889;
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_INCHES;
    
    // Safety Ramping
    private static final double MAX_ACCEL = 1.2; 
    
    // PID Gains for distance
    private static final double kP = 0.08;
    private static final double kD = 0.01;
    
    private double currentDrivePower = 0.0;
    private ElapsedTime timer = new ElapsedTime();
    private KalmanFilter kalman = new KalmanFilter();

    @Override
    public void runOpMode() {
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontTurn = hardwareMap.get(DcMotor.class, "frontTurn");

        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontTurn.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        kalman.setState(0, 0, 0);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        // 1. Move Forward 36 inches
        runToPosition(36.0);
        sleep(500);
        
        // 2. Move Backward to 0
        runToPosition(0.0);
        
        telemetry.addData("Status", "Complete");
        telemetry.update();
        sleep(2000);
    }

    private void runToPosition(double targetInches) {
        timer.reset();
        double lastTime = 0;
        double lastError = 0;

        while (opModeIsActive()) {
            double currentTime = timer.seconds();
            double deltaTime = currentTime - lastTime;
            lastTime = currentTime;

            // --- Get Distance ---
            int ticks = Math.max(Math.abs(backLeft.getCurrentPosition()), Math.abs(backRight.getCurrentPosition()));
            double measuredDistance = (ticks / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;

            // --- Kalman Update (1D) ---
            // Note: Using the 3D filter for 1D x-movement
            kalman.update(measuredDistance, 0, 0);
            double currentX = kalman.getX();

            // --- PID Control ---
            double error = targetInches - currentX;
            double derivative = (error - lastError) / (deltaTime + 1e-6);
            double targetPower = (error * kP) + (derivative * kD);
            targetPower = Range.clip(targetPower, -0.6, 0.6); // Cap power for safety

            // --- Acceleration Ramping ---
            double maxChange = MAX_ACCEL * deltaTime;
            double driveStep = Range.clip(targetPower - currentDrivePower, -maxChange, maxChange);
            currentDrivePower += driveStep;

            backLeft.setPower(currentDrivePower);
            backRight.setPower(currentDrivePower);

            telemetry.addData("Target", "%.1f", targetInches);
            telemetry.addData("Kalman X", "%.2f", currentX);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("Power", "%.2f", currentDrivePower);
            telemetry.update();

            if (Math.abs(error) < 0.5 && Math.abs(currentDrivePower) < 0.1) break;

            lastError = error;
            idle();
        }
        
        // Stop motors safely
        backLeft.setPower(0);
        backRight.setPower(0);
        currentDrivePower = 0;
    }
}
