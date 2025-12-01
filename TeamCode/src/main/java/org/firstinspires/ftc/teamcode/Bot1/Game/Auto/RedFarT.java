package org.firstinspires.ftc.teamcode.Bot1.Game.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Bot1.Game.TeleOp.Bot1FiniteState;
import org.firstinspires.ftc.teamcode.Bot1.Game.TeleOp.TelePIDConfig;
import org.firstinspires.ftc.teamcode.Bot1.Hardware.Delivery;
import org.firstinspires.ftc.teamcode.Bot1.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Bot1.Hardware.Turret;


@Disabled
@Autonomous(name = "RedFarT", group = "Auto")
public class RedFarT extends LinearOpMode {

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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



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
            turret.updateLimelightAngle(delivery.updateDistance(currentPose, "RED"), currentPose.getX(DistanceUnit.INCH), "RED");

            driveTrain.leftSpeedAdjust = 1;
            driveTrain.rightSpeedAdjust = 1;
            /*****************NEED TO BE ADJUSTED*/
            driveTrain.speed = -0.1;
            driveTrain.strafe = 0;
            driveTrain.turn =  0;

            driveTrain.leftFrontPower = Range.clip(driveTrain.speed + driveTrain.strafe - driveTrain.turn, -1, 1);
            driveTrain.rightFrontPower = Range.clip(driveTrain.speed - driveTrain.strafe + driveTrain.turn, -1, 1);
            driveTrain.leftBackPower = Range.clip(driveTrain.speed - driveTrain.strafe - driveTrain.turn, -1, 1);
            driveTrain.rightBackPower = Range.clip(driveTrain.speed + driveTrain.strafe + driveTrain.turn, -1, 1);

            driveTrain.leftFront.setPower(driveTrain.leftFrontPower * driveTrain.rightSpeedAdjust * driveTrain.leftSpeedAdjust);
            driveTrain.rightFront.setPower(driveTrain.rightFrontPower * driveTrain.rightSpeedAdjust * driveTrain.leftSpeedAdjust);
            driveTrain.leftBack.setPower(driveTrain.leftBackPower * driveTrain.rightSpeedAdjust * driveTrain.leftSpeedAdjust);
            driveTrain.rightBack.setPower(driveTrain.rightBackPower * driveTrain.rightSpeedAdjust * driveTrain.leftSpeedAdjust);

            if (result.isValid()) {
                if (result.getFiducialResults().get(0).getFiducialId() == 24) {
                    target = result.getTx();
                    if (target <= turret.RED_MAX_LEFT) {
                        turret.turret.setPower(-0.07);
                    } else if (target >= turret.RED_MAX_RIGHT) {
                        turret.turret.setPower(0.07);
                    } else {
                        turret.turret.setPower(0);
                    }
                }
            }

// If a ball is Launched,

            switch (finiteState) {
                case IDLE:
                    VELOCITY_CHECK = 0;
                    BALLS_LAUNCHED = 0;
                    break;

                case SPIN:
                    delivery.updateRPM(delivery.updateDistance(currentPose, "RED"), "RED");
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


            //during comp here to next comment, comment out

            double lVelocity = delivery.launcher.getVelocity()*60/28;
            double lCurrent =  delivery.launcher.getCurrent(CurrentUnit.MILLIAMPS);


            telemetry.addData("bot X", delivery.realX);
            telemetry.addData("bot Y", delivery.realY);
            telemetry.addData("LL Angle", target);
            telemetry.addData("Distance to Goal", delivery.DistanceToGoal);

            telemetry.addData("Target Velocity", TelePIDConfig.rpm);
            telemetry.addData("Current Velocity", lVelocity); // in rpm
            telemetry.addData("Current (A)", lCurrent);
            telemetry.addData("P Coefficient", TelePIDConfig.kP);
            telemetry.addData("I Coefficient", TelePIDConfig.kI);
            telemetry.addData("D Coefficient", TelePIDConfig.kD);
            telemetry.addData("F Coefficient", TelePIDConfig.kF);

            telemetry.addData("Loop Timer", loopTimer.milliseconds());
            telemetry.update();

            //Logcat Optional
            /*
            Log.i("FTC",  "Target=" + TelePIDConfig.rpm
                    + " Vel=" + lVelocity
                    + " V=" + battery.getVoltage()
                    + " A=" + delivery.launcher.getCurrent(CurrentUnit.MILLIAMPS)
                    + " Error=" + ((TelePIDConfig.rpm-lVelocity)/ PIDConfig.rpm)*100);
            */
            //during comp past comment to this comment, comment out
        }
    }
    public void moveForward(){
        driveTrain.speed = 1;
        driveTrain.strafe = 0;
        driveTrain.turn =  0;
    }

    public void moveStrafe(){
        driveTrain.speed = 0;
        driveTrain.strafe = 1;
        driveTrain.turn =  0;
    }

    public void Rotate(){
        driveTrain.speed = 0;
        driveTrain.strafe = 0;
        driveTrain.turn =  1;
    }
}
