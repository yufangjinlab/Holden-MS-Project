package org.firstinspires.ftc.teamcode.Bot1.Game.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Pedro.GoBildaPinpointDriver;

import java.util.Locale;


//TODO: If tuning comment out the @Disabled
@TeleOp(name="goBILDA® PinPoint Odometry Sample", group="Linear OpMode")
@Disabled
//@Disabled

public class GoBildaPinpointSampleCode extends LinearOpMode {

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        odo.setOffsets(+3.25, -6.375,DistanceUnit.INCH); ; //these are tuned for 3110-0002-0001 Product Insight #1


        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.recalibrateIMU();
        odo.resetPosAndIMU();
        double xIniPosition = 8.5;
        double yIniPosition = 85;
        double xOffset=odo.getXOffset(DistanceUnit.INCH);
        double yOffset=odo.getYOffset(DistanceUnit.INCH);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", xOffset);
        telemetry.addData("Y offset", yOffset);
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*
            Request an update from the Pinpoint odometry computer. This checks almost all outputs
            from the device in a single I2C read.
             */
            odo.update();

            /*
            Optionally, you can update only the heading of the device. This takes less time to read, but will not
            pull any other data. Only the heading (which you can pull with getHeading() or in getPosition().
             */
            //odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);


            if (gamepad1.a){
                odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
            }

            if (gamepad1.b){
                odo.recalibrateIMU(); //recalibrates the IMU without resetting position
            }

            /*
            This code prints the loop frequency of the REV Control Hub. This frequency is effected
            by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
            of time each cycle takes and finds the frequency (number of updates per second) from
            that cycle time.
             */
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;


            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
            Pose2D pos = odo.getPosition();
            Pose2D realPosition;
            double realX= xIniPosition+pos.getX(DistanceUnit.INCH);
            double realY= yIniPosition+pos.getY(DistanceUnit.INCH);
            double xBlueGoal = 137;
            double yBlueGoal = 133;
            double xRedGoal = 137;
            double yRedGoal= 11;

            double angleInDegrees = 75;
            double angleInRadians = Math.toRadians(angleInDegrees);
            double tangentValue = Math.tan(angleInRadians);
            double cosineValue = Math.cos(angleInRadians);

            double BlueXDisToGoal =realX-xBlueGoal;
            double BlueYDisToGoal = realY-yBlueGoal;
            double BlueDistanceToGoal= Math.sqrt(BlueXDisToGoal*BlueXDisToGoal+BlueYDisToGoal*BlueYDisToGoal)+4;
            double BlueDesiredShootVelLinear = BlueDistanceToGoal * Math.sqrt((386.09) / (2 * cosineValue * cosineValue * (BlueDistanceToGoal * tangentValue - 26)));
            double BlueDesiredShootVel = (BlueDesiredShootVelLinear*60)/(0.44*3.14*3.75);

            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
           double velX = odo.getVelX(DistanceUnit.INCH);
            double velY = odo.getVelY(DistanceUnit.INCH);
            double heading = odo.getHeading(AngleUnit.RADIANS);
            telemetry.addData("Blue Distance To Goal", BlueDistanceToGoal);
            telemetry.addData("Blue Desired Shooting Velocity RPM", BlueDesiredShootVel);
            telemetry.addData("realX", realX);
            telemetry.addData("realY", realY);
            telemetry.addData("Velocity", velX);
            telemetry.addData("VelocityY", velY);
            telemetry.addData("Heading", -heading*180/(3.1415926535));

            telemetry.addData("Status", odo.getDeviceStatus());

            telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            telemetry.update();

        }
    }}
