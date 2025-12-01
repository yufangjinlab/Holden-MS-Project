
package org.firstinspires.ftc.teamcode.Bot1.Game.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Bot1.Game.TeleOp.Bot1FiniteState;
import org.firstinspires.ftc.teamcode.Bot1.Hardware.Delivery;
import org.firstinspires.ftc.teamcode.Bot1.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Bot1.Hardware.Turret;


/*
 * This file includes an autonomous file from goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages mecanum wheels for robot mobility, one high-speed motor driving two "launcher wheels," and two servos
 * which feed that launcher.
 *
 * This robot starts up against the goal, move forward, and launches all three projectiles before driving away
 * off the starting line.
 *
 * This program leverages a "Finite state machine" - an Enum which captures the state of the robot
 * at any time. As it moves through the autonomous period and completes different functions,
 * it will move forward in the enum. This allows us to run the autonomous period inside of our
 * main robot "loop," continuously checking for conditions that allow us to move to the next step.
 */

@Autonomous(name="RedNear", group="RedAuto")
//@Disabled
public class RedNear extends LinearOpMode {

    DriveTrain driveTrain = new DriveTrain();
    Delivery delivery = new Delivery();
    Turret turret = new Turret();
    ElapsedTime feederTimer;
    ElapsedTime loopTimer;
    ElapsedTime PIDTimer;
    ElapsedTime SafteyTimer;
     ElapsedTime AutoRunTimer;

    private int LAUNCH_VELOCITY_AUTO_NEAR = (int) (2600 * 28)/60;
    private int MAX_LAUNCH_VELOCITY_AUTO_NEAR= LAUNCH_VELOCITY_AUTO_NEAR +45;
    private int MIN_LAUNCH_VELOCITY_AUTO_NEAR = LAUNCH_VELOCITY_AUTO_NEAR;

    private static final int PIDTimeConstant = 50;
    private static final int LAUNCH_TIME = 800;
    int BALLS_LAUNCHED = 0;
    int VELOCITY_CHECK = 0;
    double target;
    private Bot1FiniteState finiteState = Bot1FiniteState.IDLE;

    ElapsedTime autoDrivingTimer;
    private double xIniPostionRed = 111;
    private double yIniPositionRed = 17;
    int finalParkCheck =0;


    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain.init(hardwareMap);
        delivery.init(hardwareMap);
        turret.init(hardwareMap);
        finiteState = Bot1FiniteState.WAIT_DRIVE;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        loopTimer = new ElapsedTime();
        feederTimer = new ElapsedTime();
        PIDTimer = new ElapsedTime();
        SafteyTimer = new ElapsedTime();
        AutoRunTimer = new ElapsedTime();

        autoDrivingTimer = new ElapsedTime();

        Pose2D previousPose = null;
        double heading;

        turret.limelight.start();
        driveTrain.pinpoint.resetPosAndIMU();
        driveTrain.pinpoint.update();
        Pose2D currentPose= driveTrain.pinpoint.getPosition();
        delivery.xIniPosition = xIniPostionRed+currentPose.getX(DistanceUnit.INCH);//-34;
        delivery.yIniPosition = yIniPositionRed+currentPose.getX(DistanceUnit.INCH);
        telemetry.addData("Initial Position", currentPose);
        telemetry.update();
        waitForStart();

        AutoRunTimer.reset();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            loopTimer.reset();

            LLResult result = turret.limelight.getLatestResult();

//            /************* The turret keeps changing. NOT A GOOD Way for AUTO**************/
            /*********
             if (result.isValid()) {
             if (result.getFiducialResults().get(0).getFiducialId() == 24) {
             target = result.getTx();
             if (target <= turret.RED_MAX_RIGHT+2.5) {
             turret.turret.setPower(-0.07);
             } else if (target >= turret.RED_MAX_LEFT-0.5) {
             turret.turret.setPower(0.07);
             } else {
             turret.turret.setPower(0);
             }
             }
             }

             turret.updateLimelightAngle(delivery.updateDistance(currentPose, "RED"), currentPose.getX(DistanceUnit.INCH), "RED");
             ********************/

            /*****************NEED TO BE ADJUSTED*/
            switch (finiteState) {
                case WAIT_DRIVE:
                    autoDrivingTimer.reset();
                    finiteState = Bot1FiniteState.DRIVING_TO_LAUNCH_POSE;
                    break;

                case DRIVING_TO_LAUNCH_POSE:
                    if (autoDrivingTimer.milliseconds() <725) {
                        moveBackward();
                        mecanumDrive();
                        telemetry.addData("Status", "Robot move");
                    } else {
                        autoDrivingTimer.reset();
                        stopMotors();
                        finiteState = Bot1FiniteState.IDLE;
                        telemetry.addData("Status", "Robot Stopped");
                    }
                    break;

                case IDLE:
                    if(BALLS_LAUNCHED>=3){
                        VELOCITY_CHECK = 0;
                        BALLS_LAUNCHED = 0;
                    }
                    /* if use the camera, change the 500 timing to 1500*/
                    else if (autoDrivingTimer.milliseconds() > 500) {
                        currentPose = driveTrain.pinpoint.getPosition();
                        driveTrain.pinpoint.update();
                        autoDrivingTimer.reset();
                        finiteState = Bot1FiniteState.SPIN;
                    }
                    break;
                /********************* This will call the automatic launching speed from delivery
                 case SPIN:
                 delivery.updateRPM(delivery.updateDistance(currentPose, "RED"));
                 delivery.launcher.setVelocity(delivery.TARGET_LAUNCH_VELOCITY);
                 if ((delivery.MAX_LAUNCH_VELOCITY > delivery.launcher.getVelocity()) && (delivery.launcher.getVelocity() > delivery.MIN_LAUNCH_VELOCITY)) {
                 VELOCITY_CHECK += 1;
                 if (VELOCITY_CHECK == 1) {
                 PIDTimer.reset();
                 }
                 if (PIDTimer.milliseconds() > PIDTimeConstant) {
                 finiteState = Bot1FiniteState.LAUNCH_START;
                 previousPose = currentPose;
                 feederTimer.reset();
                 }
                 } else {
                 VELOCITY_CHECK = 0;
                 }
                 break;
                 */
                case SPIN:
                    //delivery.updateRPM(delivery.updateDistance(currentPose, "RED"));
                    delivery.launcher.setVelocity(LAUNCH_VELOCITY_AUTO_NEAR);
                    if ((MAX_LAUNCH_VELOCITY_AUTO_NEAR > delivery.launcher.getVelocity()) && (delivery.launcher.getVelocity() > MIN_LAUNCH_VELOCITY_AUTO_NEAR)) {
                        VELOCITY_CHECK += 1;
                        if (VELOCITY_CHECK == 1) {
                            PIDTimer.reset();
                        }
                        if (PIDTimer.milliseconds() > PIDTimeConstant) {
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
                            (currentPose.getY(DistanceUnit.INCH) > previousPose.getY(DistanceUnit.INCH) + 1)) {
                        VELOCITY_CHECK = 0;
                        finiteState = Bot1FiniteState.SPIN;
                    } else if (feederTimer.milliseconds() > 300) {
                        delivery.leftFeeder.setPower(0.5);
                        delivery.rightFeeder.setPower(0.5);
                        finiteState = Bot1FiniteState.LAUNCHING;
                        BALLS_LAUNCHED +=1;
                        feederTimer.reset();
                    }
                    break;

                case LAUNCHING:
                    if (feederTimer.milliseconds() > 180) {
                        delivery.rightFeeder.setPower(0);
                        delivery.leftFeeder.setPower(0);
                    }

                    if (feederTimer.milliseconds() > LAUNCH_TIME) {
                        if (BALLS_LAUNCHED >= 3) {
                            delivery.launcher.setPower(0);
                            finiteState = Bot1FiniteState.DRIVING_AWAY_FROM_LAUNCH;
                            autoDrivingTimer.reset();
                        } else {
                            finiteState = Bot1FiniteState.REVERSE_FEEDER;
                            feederTimer.reset();
                        }
                    }
                    break;

                case REVERSE_FEEDER:
                    if (feederTimer.milliseconds() > 50) {
                        delivery.rightFeeder.setPower(-1);
                        delivery.leftFeeder.setPower(-1);
                        finiteState = Bot1FiniteState.STOP_FEEDER;
                        feederTimer.reset();
                    }
                    break;

                case STOP_FEEDER:
                    if (feederTimer.milliseconds() > 50) {
                        delivery.rightFeeder.setPower(0);
                        delivery.leftFeeder.setPower(0);
                        finiteState = Bot1FiniteState.SPIN;
                    }
                    break;

                case DRIVING_AWAY_FROM_LAUNCH:
                    delivery.rightFeeder.setPower(0);
                    delivery.leftFeeder.setPower(0);
                    delivery.launcher.setPower(0);
                    turret.turret.setPower(0);
                    if (autoDrivingTimer.milliseconds()<2000) {
                        moveStrafeRight();
                        // moveStrafeRightForward();
                        mecanumDrive();
                    } else {
                        stopMotors();
                        finiteState = Bot1FiniteState.COMPLETE;
                        autoDrivingTimer.reset();
                    }
                    break;

                case COMPLETE:
                    if(autoDrivingTimer.milliseconds() > 1000){

                        currentPose = driveTrain.pinpoint.getPosition();
                        driveTrain.pinpoint.update();
                        delivery.xIniPosition = xIniPostionRed+currentPose.getX(DistanceUnit.INCH);
                        delivery.yIniPosition = yIniPositionRed+currentPose.getY(DistanceUnit.INCH)+1;
//                    RobotState.robotStartPosition = currentPose;
                        // telemetry.addData("RobortStartPosition", RobotState.robotStartPosition);
                        telemetry.addData("current X Position", delivery.xIniPosition);
                        telemetry.addData("current Y Position", delivery.yIniPosition);
                        telemetry.addData("finiteState", finiteState);
                        telemetry.update();
                        //autoDrivingTimer.reset();
                    }
                    break;

                case FINAL_PARK:
                    delivery.launcher.setPower(0);
                    delivery.leftFeeder.setPower(0);
                    delivery.rightFeeder.setPower(0);
                    turret.turret.setPower(0);
                    if (autoDrivingTimer.milliseconds()<2000) {
                        moveStrafeRight();
                        mecanumDrive();
                    }
                    else{
                        stopMotors();
                        finiteState = Bot1FiniteState.COMPLETE;
                        autoDrivingTimer.reset();
                    }
                    telemetry.addData("finiteState", finiteState);
                    telemetry.update();
                    break;


            }

            if ((AutoRunTimer.milliseconds()>=27000)&&(finiteState!=Bot1FiniteState.COMPLETE)&&(finalParkCheck==0)) {
                stopMotors();
                finalParkCheck=1;
                finiteState = Bot1FiniteState.FINAL_PARK;
                autoDrivingTimer.reset();
            }
            //during comp here to next comment, comment out

            // double lVelocity = delivery.launcher.getVelocity() * 60 / 28;
            //double lCurrent = delivery.launcher.getCurrent(CurrentUnit.MILLIAMPS);

            // telemetry.addData("bot X", delivery.realX);
            //   telemetry.addData("bot Y", delivery.realY);
            //   telemetry.addData("LL Angle", target);
            // telemetry.addData("Distance to Goal", delivery.DistanceToGoal);

            //telemetry.addData("Target Velocity", TelePIDConfig.rpm);
            //telemetry.addData("Current Velocity", lVelocity); // in rpm
            //telemetry.addData("robotStartPosition", RobotState.robotStartPosition);
            //telemetry.addData("current Position", currentPose);
            //telemetry.update();
            /*
                telemetry.addData("Current (A)", lCurrent);
                telemetry.addData("P Coefficient", TelePIDConfig.kP);
                telemetry.addData("I Coefficient", TelePIDConfig.kI);
                telemetry.addData("D Coefficient", TelePIDConfig.kD);
                telemetry.addData("F Coefficient", TelePIDConfig.kF);

                telemetry.addData("Loop Timer", loopTimer.milliseconds());
                telemetry.update();
*/
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
        driveTrain.speed = -1;
        driveTrain.strafe = 0;
        driveTrain.turn =  0;
    }

    public void moveBackward(){
        driveTrain.speed = 1;
        driveTrain.strafe = 0;
        driveTrain.turn =  0;
    }

    public void moveStrafeLeft(){
        driveTrain.speed = 0;
        driveTrain.strafe = 1;
        driveTrain.turn =  0;
    }

    public void moveStrafeRightForward(){
        driveTrain.speed = -1;
        driveTrain.strafe = -1;
        driveTrain.turn =  0;
    }

    public void moveStrafeRight(){
        driveTrain.speed = 0;
        driveTrain.strafe = -1;
        driveTrain.turn = 0;
    }

    public void Rotate(){
        driveTrain.speed = 0;
        driveTrain.strafe = 0;
        driveTrain.turn =  1;
    }

    public void mecanumDrive(){
        double leftSpeedAdjust = 0.75;
        double rightSpeedAdjust = 0.5;
        driveTrain.leftFrontPower = Range.clip(driveTrain.speed + driveTrain.strafe - driveTrain.turn, -1, 1);
        driveTrain.rightFrontPower = Range.clip(driveTrain.speed - driveTrain.strafe + driveTrain.turn, -1, 1);
        driveTrain.leftBackPower = Range.clip(driveTrain.speed - driveTrain.strafe - driveTrain.turn, -1, 1);
        driveTrain.rightBackPower = Range.clip(driveTrain.speed + driveTrain.strafe + driveTrain.turn, -1, 1);

        driveTrain.leftFront.setPower(driveTrain.leftFrontPower * rightSpeedAdjust * leftSpeedAdjust);
        driveTrain.rightFront.setPower(driveTrain.rightFrontPower * rightSpeedAdjust * leftSpeedAdjust);
        driveTrain.leftBack.setPower(driveTrain.leftBackPower * rightSpeedAdjust * leftSpeedAdjust);
        driveTrain.rightBack.setPower(driveTrain.rightBackPower * rightSpeedAdjust * leftSpeedAdjust);

    }
    public void stopMotors() {
        driveTrain.leftFront.setPower(0);
        driveTrain.rightFront.setPower(0);
        driveTrain.leftBack.setPower(0);
        driveTrain.rightBack.setPower(0);
    }

}




