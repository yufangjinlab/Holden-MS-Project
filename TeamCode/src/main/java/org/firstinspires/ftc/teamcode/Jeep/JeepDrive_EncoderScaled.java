package org.firstinspires.ftc.teamcode.Jeep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "JeepDrive_EncoderScaled", group = "Jeep")
public class JeepDrive_EncoderScaled extends LinearOpMode {

    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotor frontTurn;

    // ======== DRIVETRAIN CONSTANTS ========
    private static final double TICKS_PER_REV = 2000;

    // GEAR PROTECTION: Higher values = faster/snappier, Lower values = smoother/safer
    // 1.2 means it takes ~0.8 seconds to go from 0 to full power (1.0 / 1.2 = 0.83)
    private static final double MAX_ACCEL = 1.2;

    // 48mm diameter encoder wheel (approx 1.889 inches)
    private static final double WHEEL_DIAMETER_INCHES = 1.889;
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_INCHES;

    // ======== CALIBRATION & STATE ========
    private static double ENCODER_SCALE = 1.0;
    private double currentDrivePower = 0.0;
    private double currentSteerPower = 0.0;
    
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize hardware
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontTurn = hardwareMap.get(DcMotor.class, "frontTurn");

        // Set directions - assuming mirrored rear motors
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontTurn.setDirection(DcMotorSimple.Direction.REVERSE);

        // GEAR PROTECTION: Use FLOAT instead of BRAKE
        // This prevents the "mechanical shock" when power reaches zero.
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        // Steering usually needs BRAKE to stay in place, but we will ramp it too.
        frontTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized - GEAR PROTECTION ENABLED");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            double deltaTime = timer.seconds();
            timer.reset();

            // ===== DRIVE WITH TIME-BASED RAMPING =====
            double targetDrive = -gamepad1.left_stick_y;
            double scaledTargetDrive = Math.pow(targetDrive, 3);

            // Calculate how much we can change power based on the time since last loop
            double maxChange = MAX_ACCEL * deltaTime;

            // Constrain the change to the limit
            double driveError = scaledTargetDrive - currentDrivePower;
            double driveStep = Range.clip(driveError, -maxChange, maxChange);
            currentDrivePower += driveStep;

            backLeft.setPower(currentDrivePower);
            backRight.setPower(currentDrivePower);

            // ===== STEERING WITH TIME-BASED RAMPING =====
            double targetSteer = gamepad1.right_stick_x;
            
            double steerError = targetSteer - currentSteerPower;
            double steerStep = Range.clip(steerError, -maxChange, maxChange);
            currentSteerPower += steerStep;
            
            frontTurn.setPower(currentSteerPower);

            // ===== ENCODER MATH =====
            // FIX: If distance was off by half, it's because we averaged a motor with no encoder.
            // We'll use the maximum of the two positions to catch whichever port has the dead-wheel.
            int leftTicks = Math.abs(backLeft.getCurrentPosition());
            int rightTicks = Math.abs(backRight.getCurrentPosition());
            int maxTicks = Math.max(leftTicks, rightTicks);

            double rawDistanceInches = (maxTicks / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;
            double correctedDistanceInches = rawDistanceInches * ENCODER_SCALE;

            // Live tuning for calibration (hold bumpers to change faster)
            if (gamepad1.dpad_up) ENCODER_SCALE += 0.0001;
            if (gamepad1.dpad_down) ENCODER_SCALE -= 0.0001;

            telemetry.addData("Drive Power", "%.2f", currentDrivePower);
            telemetry.addData("Steer Power", "%.2f", currentSteerPower);
            telemetry.addData("Raw Ticks (L/R)", "%d / %d", leftTicks, rightTicks);
            telemetry.addData("Raw Distance (in)", "%.2f", rawDistanceInches);
            telemetry.addData("Scaled Distance (in)", "%.2f", correctedDistanceInches);
            telemetry.addData("Encoder Scale", "%.5f", ENCODER_SCALE);
            telemetry.update();
        }
    }
}
