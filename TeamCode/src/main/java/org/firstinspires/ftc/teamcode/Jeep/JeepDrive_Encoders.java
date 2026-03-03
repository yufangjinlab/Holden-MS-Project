package org.firstinspires.ftc.teamcode.Jeep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "JeepDrive_ExtEncoderTest", group = "Jeep")
public class JeepDrive_Encoders extends LinearOpMode {

    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx frontTurn;

    // ======== EXTERNAL ENCODER CONSTANTS ========
    private static final double TICKS_PER_REV = 2000.0;
    private static final double ENCODER_DIAMETER_METERS = 0.048;
    private static final double ENCODER_CIRCUMFERENCE =
            Math.PI * ENCODER_DIAMETER_METERS;

    @Override
    public void runOpMode() {

        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight"); // external encoder plugged here
        frontTurn = hardwareMap.get(DcMotorEx.class, "frontTurn");

        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset ONLY external encoder (on backRight port)
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Run motors open loop
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            // ======================
            // DRIVE CONTROL
            // ======================
            double drive = -gamepad1.left_stick_y;  // negative so forward is positive
            drive = Math.pow(drive, 3);

            backLeft.setPower(drive);
            backRight.setPower(drive);

            // ======================
            // STEERING CONTROL
            // ======================
            double steer = gamepad1.right_stick_x;
            frontTurn.setPower(steer);

            // ======================
            // EXTERNAL ENCODER READ
            // ======================
            int encoderTicks = backRight.getCurrentPosition();

            double distanceMeters =
                    (encoderTicks / TICKS_PER_REV) * ENCODER_CIRCUMFERENCE;

            double velocityTicks = backRight.getVelocity(); // ticks/sec

            double velocityMetersPerSec =
                    (velocityTicks / TICKS_PER_REV) * ENCODER_CIRCUMFERENCE;

            telemetry.addLine("=== EXTERNAL ENCODER TEST ===");
            telemetry.addData("Raw Ticks", encoderTicks);
            telemetry.addData("Distance (m)", distanceMeters);
            telemetry.addData("Distance (ft)", distanceMeters * 3.28084);
            telemetry.addData("Velocity (ticks/sec)", velocityTicks);
            telemetry.addData("Velocity (m/s)", velocityMetersPerSec);
            telemetry.update();
        }
    }
}