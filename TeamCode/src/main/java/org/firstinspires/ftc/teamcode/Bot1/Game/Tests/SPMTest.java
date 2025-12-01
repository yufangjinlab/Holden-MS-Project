package org.firstinspires.ftc.teamcode.Bot1.Game.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Gamepad;
@Disabled
@TeleOp(name="SPMTest", group="Test")
public class SPMTest extends LinearOpMode {

    private CRServoImplEx servo;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        servo = hardwareMap.get(CRServoImplEx.class, "servo");

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();


        waitForStart();
        servo.setPower(0);
//

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.a){
                servo.setPower(1);
            }
            if (gamepad1.b){
                servo.setPower(-1);
            }
            if (gamepad1.y){
                servo.setPower(0);
            }
            currentGamepad1.copy(gamepad1);
            previousGamepad1.copy(currentGamepad1);
            // Show the elapsed game time and wheel power.

            telemetry.update();
        }
    }
}