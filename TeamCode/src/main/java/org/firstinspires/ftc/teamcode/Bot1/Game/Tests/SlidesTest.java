package org.firstinspires.ftc.teamcode.Bot1.Game.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
@Disabled
@TeleOp(name="SlidesTest", group="Test")
public class SlidesTest extends LinearOpMode {

    private DcMotorEx gun;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        gun = hardwareMap.get(DcMotorEx.class, "gun");

        gun.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        gun.setPower(0);

        gun.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        gun.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gun.setDirection(DcMotorEx.Direction.FORWARD);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();


        waitForStart();
//

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.a && !previousGamepad1.a){
                gun.setTargetPosition(gun.getCurrentPosition()+50);
                gun.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gun.setPower(1);
            }

            if (currentGamepad1.b && !previousGamepad1.b){
                gun.setTargetPosition(gun.getCurrentPosition()-50);
                gun.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gun.setPower(1);
            }

            previousGamepad1.copy(currentGamepad1);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Motor Position", gun.getCurrentPosition());
            telemetry.update();
        }
    }
}