package org.firstinspires.ftc.teamcode.Bot1.Game.TeleOp;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Bot1.Hardware.Delivery;
import org.firstinspires.ftc.teamcode.Bot1.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Bot1.Hardware.Turret;


@TeleOp(name = "Bot 1 TeleOp: Blue", group = "TeleOp")
public class Bot1BlueTeleOp extends LinearOpMode {
    private VoltageSensor battery;
    DriveTrain driveTrain = new DriveTrain();
    Delivery delivery = new Delivery();
    Turret turret = new Turret();
    ElapsedTime feederTimer;
    ElapsedTime loopTimer;
    ElapsedTime PIDTimer;
    private static final int PIDTimeConstant = 50;
    private static final int LAUNCH_TIME = 800;
    int BALLS_LAUNCHED = 0;
    int VELOCITY_CHECK = 0;
    double target;
    private Bot1FiniteState finiteState = Bot1FiniteState.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain.init(hardwareMap);
        delivery.init(hardwareMap);
        turret.init(hardwareMap);

        battery = hardwareMap.voltageSensor.iterator().next();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        loopTimer = new ElapsedTime();
        feederTimer = new ElapsedTime();
        PIDTimer = new ElapsedTime();

        Pose2D previousPose = null;

        turret.limelight.start();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            loopTimer.reset();

            LLResult result = turret.limelight.getLatestResult();
            Pose2D currentPose = driveTrain.pinpoint.getPosition();
            driveTrain.pinpoint.update();
            turret.updateLimelightAngle(delivery.updateDistance(currentPose, "BLUE"),currentPose.getX(DistanceUnit.INCH), "BLUE");


            if (gamepad1.left_trigger > 0) {
                driveTrain.leftSpeedAdjust = 0.7;
            } else {
                driveTrain.leftSpeedAdjust = 1;
            }

            if (gamepad1.right_trigger > 0) {
                driveTrain.rightSpeedAdjust = 0.5;
            } else {
                driveTrain.rightSpeedAdjust = 1;
            }

            driveTrain.speed = gamepad1.left_stick_y;
            driveTrain.strafe = -gamepad1.left_stick_x;
            driveTrain.turn = - gamepad1.right_stick_x * 0.75;

            driveTrain.leftFrontPower = Range.clip(driveTrain.speed + driveTrain.strafe - driveTrain.turn, -1, 1);
            driveTrain.rightFrontPower = Range.clip(driveTrain.speed - driveTrain.strafe + driveTrain.turn, -1, 1);
            driveTrain.leftBackPower = Range.clip(driveTrain.speed - driveTrain.strafe - driveTrain.turn, -1, 1);
            driveTrain.rightBackPower = Range.clip(driveTrain.speed + driveTrain.strafe + driveTrain.turn, -1, 1);

            driveTrain.leftFront.setPower(driveTrain.leftFrontPower * driveTrain.rightSpeedAdjust * driveTrain.leftSpeedAdjust);
            driveTrain.rightFront.setPower(driveTrain.rightFrontPower * driveTrain.rightSpeedAdjust * driveTrain.leftSpeedAdjust);
            driveTrain.leftBack.setPower(driveTrain.leftBackPower * driveTrain.rightSpeedAdjust * driveTrain.leftSpeedAdjust);
            driveTrain.rightBack.setPower(driveTrain.rightBackPower * driveTrain.rightSpeedAdjust * driveTrain.leftSpeedAdjust);

            if (result.isValid()){
                if (result.getFiducialResults().get(0).getFiducialId() == 20){
                    target = result.getTx();
                    if(target <= turret.BLUE_MAX_RIGHT - 8){
                        turret.turret.setPower(-0.11);
                    } else if (target >= turret.BLUE_MAX_LEFT + 8){
                        turret.turret.setPower(0.13);
                    }

                    if(target <= turret.BLUE_MAX_RIGHT && target > turret.BLUE_MAX_RIGHT - 8){
                        turret.turret.setPower(-0.05);
                    } else if (target >= turret.BLUE_MAX_LEFT && target < turret.BLUE_MAX_LEFT + 8){
                        turret.turret.setPower(0.07);
                    } else if (target < turret.BLUE_MAX_LEFT && target > turret.BLUE_MAX_RIGHT){
                        turret.turret.setPower(0);
                    }
                } else if(gamepad1.left_bumper){
                    turret.turret.setPower(-0.3);
                } else if(gamepad1.right_bumper){
                    turret.turret.setPower(0.3);
                }
            } else if(gamepad1.left_bumper){
                turret.turret.setPower(-0.3);
            } else if(gamepad1.right_bumper){
                turret.turret.setPower(0.3);
            } else {
                turret.turret.setPower(0);
            }

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if(currentGamepad1.a && !previousGamepad1.a){
                BALLS_LAUNCHED += 1;
                finiteState = Bot1FiniteState.SPIN;
            }

            if(currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                delivery.launcher.setPower(0);
                finiteState = Bot1FiniteState.IDLE;
            }

            if(currentGamepad1.start && !previousGamepad1.start){
                driveTrain.pinpoint.resetPosAndIMU();
                delivery.xIniPosition = 8.5;
                delivery.yIniPosition = 9;
            }

            switch (finiteState) {
                case IDLE:
                    VELOCITY_CHECK = 0;
                    BALLS_LAUNCHED = 0;
                    break;

                case SPIN:
                    delivery.updateRPM(delivery.updateDistance(currentPose, "BLUE"), "BLUE");
                    delivery.launcher.setVelocity(delivery.TARGET_LAUNCH_VELOCITY);
                    if ((delivery.MAX_LAUNCH_VELOCITY > delivery.launcher.getVelocity())&& (delivery.launcher.getVelocity() > delivery.MIN_LAUNCH_VELOCITY)) {
                        VELOCITY_CHECK += 1;
                        if (VELOCITY_CHECK == 1){
                            PIDTimer.reset();
                        }
                        if (PIDTimer.milliseconds() > PIDTimeConstant){
                            finiteState = Bot1FiniteState.LAUNCH_START;
                            previousPose = currentPose;
                            feederTimer.reset();
                        }
                    } else {
                        VELOCITY_CHECK = 0;
                    }
                    break;

                case LAUNCH_START:
                    if ((currentPose.getX(DistanceUnit.INCH) < previousPose.getX(DistanceUnit.INCH) - 1) ||
                            (currentPose.getX(DistanceUnit.INCH) > previousPose.getX(DistanceUnit.INCH) + 1) ||
                            (currentPose.getY(DistanceUnit.INCH) < previousPose.getY(DistanceUnit.INCH) - 1) ||
                            (currentPose.getY(DistanceUnit.INCH) > previousPose.getY(DistanceUnit.INCH) + 1)){
                        VELOCITY_CHECK = 0;
                        finiteState = Bot1FiniteState.SPIN;
                    } else if (feederTimer.milliseconds() > 300) {
                        delivery.leftFeeder.setPower(0.5);
                        delivery.rightFeeder.setPower(0.5);
                        finiteState = Bot1FiniteState.LAUNCHING;
                        feederTimer.reset();
                    }
                    break;

                case LAUNCHING:
                    if (feederTimer.milliseconds() > 150){
                        delivery.rightFeeder.setPower(0);
                        delivery.leftFeeder.setPower(0);
                    }

                    if (feederTimer.milliseconds() > LAUNCH_TIME) {
                        if (BALLS_LAUNCHED >= 3) {
                            delivery.launcher.setPower(0);
                            finiteState = Bot1FiniteState.IDLE;
                        } else {
                            finiteState = Bot1FiniteState.REVERSE_FEEDER;
                            feederTimer.reset();
                        }
                    }
                    break;

                case REVERSE_FEEDER:
                    if (feederTimer.milliseconds() > 50){
                        delivery.rightFeeder.setPower(-1);
                        delivery.leftFeeder.setPower(-1);
                        finiteState = Bot1FiniteState.STOP_FEEDER;
                        feederTimer.reset();
                    }
                    break;

                case STOP_FEEDER:
                    if (feederTimer.milliseconds() > 50){
                        delivery.rightFeeder.setPower(0);
                        delivery.leftFeeder.setPower(0);
                    }
                    break;
            }

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            //during comp here to next comment, comment out

//            double lVelocity = delivery.launcher.getVelocity()*60/28;
//            double lCurrent =  delivery.launcher.getCurrent(CurrentUnit.MILLIAMPS);
//            double lVoltage  = battery.getVoltage();
//
            telemetry.addData("bot X", delivery.realX);
//            telemetry.addData("bot Y", delivery.realY);
            telemetry.addData("LL Angle", target);
            telemetry.addData("Distance to Goal", delivery.DistanceToGoal);
//
//            telemetry.addData("Target Velocity", TelePIDConfig.rpm);
//            telemetry.addData("Current Velocity", lVelocity); // in rpm
//            telemetry.addData("Current (A)", lCurrent);
//            telemetry.addData("Voltage (V)", lVoltage);
//            telemetry.addData("P Coefficient", TelePIDConfig.kP);
//            telemetry.addData("I Coefficient", TelePIDConfig.kI);
//            telemetry.addData("D Coefficient", TelePIDConfig.kD);
//            telemetry.addData("F Coefficient", TelePIDConfig.kF);
//
            telemetry.addData("Loop Timer", loopTimer.milliseconds());
            telemetry.update();
//
//            //Logcat Optional
//            Log.i("FTC",  "Target=" + TelePIDConfig.rpm
//                    + " Vel=" + lVelocity
//                    + " V=" + battery.getVoltage()
//                    + " A=" + delivery.launcher.getCurrent(CurrentUnit.MILLIAMPS)
//                    + " Error=" + ((TelePIDConfig.rpm-lVelocity)/PIDConfig.rpm)*100);

            //during comp past comment to this comment, comment out
        }
    }
}
