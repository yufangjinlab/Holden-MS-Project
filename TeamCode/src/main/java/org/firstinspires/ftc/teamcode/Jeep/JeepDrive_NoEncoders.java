package org.firstinspires.ftc.teamcode.Jeep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "JeepDrive_NoEncoders", group = "Jeep")
public class JeepDrive_NoEncoders extends LinearOpMode {

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontTurn;

    @Override
    public void runOpMode() {

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontTurn = hardwareMap.get(DcMotor.class, "frontTurn");

        // Reverse one side if needed
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Brake mode for better stopping
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            // ======================
            // DRIVE CONTROL
            // ======================
            double drive = gamepad1.left_stick_y;
            drive = Math.pow(drive, 3);  // smoother control

            backLeft.setPower(drive);
            backRight.setPower(drive);

            // ======================
            // STEERING CONTROL
            // ======================
            double steer = gamepad1.right_stick_x;

            // Reduce steering speed so it doesn't slam
            double steeringPower = steer ;

            frontTurn.setPower(steeringPower);

            // ======================
            // TELEMETRY
            // ======================
            telemetry.addData("Drive", drive);
            telemetry.addData("Steer Power", steeringPower);
            telemetry.update();
        }
    }
}
