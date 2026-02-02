package org.firstinspires.ftc.teamcode.Kalman;

import com.pedropathing.geometry.Pose;

public class KalmanTest {

    private Pose lastPose = null;

    private final KalmanFilter filter = new KalmanFilter();

    public Pose update(Pose measuredPose) {

        // First call: initialize filter
        if (lastPose == null) {
            lastPose = measuredPose;
            filter.setState(
                    measuredPose.getX(),
                    measuredPose.getY(),
                    measuredPose.getHeading()
            );
            return measuredPose;
        }

        // ----- Compute FIELD frame delta -----
        double dxField = measuredPose.getX() - lastPose.getX();
        double dyField = measuredPose.getY() - lastPose.getY();
        double dHeading = measuredPose.getHeading() - lastPose.getHeading();

        // ----- Convert to ROBOT frame delta (CRITICAL) -----
        double h = lastPose.getHeading();

        double dxRobot =  dxField * Math.cos(h) + dyField * Math.sin(h);
        double dyRobot = -dxField * Math.sin(h) + dyField * Math.cos(h);

        // ----- EKF Predict with robot motion -----
        filter.predict(dxRobot, dyRobot, dHeading);

        // ----- EKF Update with field measurement -----
        filter.update(
                measuredPose.getX(),
                measuredPose.getY(),
                measuredPose.getHeading()
        );

        lastPose = measuredPose;

        return new Pose(
                filter.getX(),
                filter.getY(),
                filter.getHeading()
        );
    }
}
