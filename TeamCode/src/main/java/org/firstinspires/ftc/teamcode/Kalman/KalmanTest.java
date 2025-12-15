package org.firstinspires.ftc.teamcode.Kalman;

import com.pedropathing.geometry.Pose;

public class KalmanTest {

    private Pose lastPose = new Pose(0, 0, 0);

    private final KalmanFilter filter = new KalmanFilter(
            0.05,
            0.5
    );

    public Pose update(Pose measuredPose) {

        double dx = measuredPose.getX() - lastPose.getX();
        double dy = measuredPose.getY() - lastPose.getY();
        double dHeading = measuredPose.getHeading() - lastPose.getHeading();

        filter.predict(dx, dy, dHeading);

        lastPose = measuredPose;

        filter.update(
                measuredPose.getX(),
                measuredPose.getY(),
                measuredPose.getHeading()
        );

        return new Pose(
                filter.getX(),
                filter.getY(),
                filter.getHeading()
        );
    }
}
