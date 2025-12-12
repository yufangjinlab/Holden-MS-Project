package org.firstinspires.ftc.teamcode.Kalman;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.pedropathing.geometry.Pose;

public class KalmanTest {

    private final DcMotorEx leftEncoder, rightEncoder;
    private final BNO055IMU imu;

    private int lastLeft = 0, lastRight = 0;
    private double lastHeading = 0;

    private final double TICKS_TO_INCHES = 0.00123;  // ADJUST FOR YOUR ROBOT

    private final KalmanFilter filter = new KalmanFilter(
            0.05,   // process noise
            0.5     // measurement noise
    );

    public KalmanTest(HardwareMap hardwareMap) {

        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftOdo");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightOdo");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(params);

        lastLeft = leftEncoder.getCurrentPosition();
        lastRight = rightEncoder.getCurrentPosition();
        lastHeading = getIMUHeading();
    }

    private double getIMUHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public Pose update() {
        int currLeft = leftEncoder.getCurrentPosition();
        int currRight = rightEncoder.getCurrentPosition();

        double dL = (currLeft - lastLeft) * TICKS_TO_INCHES;
        double dR = (currRight - lastRight) * TICKS_TO_INCHES;

        double dHeading = getIMUHeading() - lastHeading;

        double dx = (dL + dR) / 2.0;
        double dy = 0; // 2-wheel odometry has no strafe info

        filter.predict(dx, dy, dHeading);

        lastLeft = currLeft;
        lastRight = currRight;
        lastHeading = getIMUHeading();

        // Update only heading (no external X/Y sources)
        filter.update(
                filter.getX(),
                filter.getY(),
                getIMUHeading()
        );

        return new Pose(
                filter.getX(),
                filter.getY(),
                filter.getHeading()
        );
    }
}
