package org.firstinspires.ftc.teamcode.Jeep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Left Front Motor 5s Test", group = "Test")
public class LeftFrontMotorTest extends LinearOpMode {

    private DcMotor leftFront;

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");

        // Optional but recommended
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (opModeIsActive()) {

            // Forward for 5 seconds
            leftFront.setPower(0.5);
            sleep(5000);

            // Stop briefly
            leftFront.setPower(0);
            sleep(1000);

            // Reverse for 5 seconds
            leftFront.setPower(-0.5);
            sleep(5000);

            // Stop
            leftFront.setPower(0);
        }
    }
}
