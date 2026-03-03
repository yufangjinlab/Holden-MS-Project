package org.firstinspires.ftc.teamcode.Jeep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "JeepDrive_EncoderScaled", group = "Jeep")
public class JeepDrive_EncoderScaled extends LinearOpMode {

    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotor frontTurn;

    // ======== DRIVETRAIN CONSTANTS ========
    private static final double TICKS_PER_REV = 2000;

    // 7 inch radius wheel
    private static final double WHEEL_RADIUS_INCHES = 1.889;
    private static final double WHEEL_CIRCUMFERENCE =
            2 * Math.PI * WHEEL_RADIUS_INCHES;

    // ======== TUNE THIS VALUE ========
    private static double ENCODER_SCALE = 1.0;

    @Override
    public void runOpMode() {

        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontTurn = hardwareMap.get(DcMotor.class, "frontTurn");

        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontTurn.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            // ===== DRIVE =====
            double drive = gamepad1.left_stick_y;
            drive = Math.pow(drive, 3);

            backLeft.setPower(drive);
            backRight.setPower(drive);

            // ===== STEERING =====
            double steer = gamepad1.right_stick_x;
            frontTurn.setPower(steer);

            // ===== ENCODER MATH =====
            int leftTicks = backLeft.getCurrentPosition();
            int rightTicks = backRight.getCurrentPosition();

            double avgTicks = (leftTicks + rightTicks) / 2.0;

            double rawDistanceInches =
                    (avgTicks / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;

            double correctedDistanceInches =
                    rawDistanceInches * ENCODER_SCALE;

            telemetry.addData("Raw Distance (in)", rawDistanceInches);
            telemetry.addData("Corrected Distance (in)", correctedDistanceInches);
            telemetry.addData("Encoder Scale", ENCODER_SCALE);
            telemetry.update();
        }
    }
}
