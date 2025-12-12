package org.firstinspires.ftc.teamcode.Kalman;

import static org.firstinspires.ftc.teamcode.Pedro.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.Pedro.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.Pedro.pedroPathing.Tuning.follower;
//import static org.firstinspires.ftc.teamcode.Kalman.TriTest.telemetryM;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class TriTest extends OpMode {
    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    private KalmanTest kalman;   // NEW

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose interPose = new Pose(24, -24, Math.toRadians(90));
    private final Pose endPose   = new Pose(24, 24,  Math.toRadians(45));

    private PathChain triangle;

    @Override
    public void init() {
        kalman = new KalmanTest(hardwareMap);   // NEW
    }

    @Override
    public void init_loop() {
        telemetryM.debug("Kalman Filtering Enabled (Odometry+IMU).");
        telemetryM.update(telemetry);

        follower.update();
        follower.setPose(kalman.update()); // NEW

        drawOnlyCurrent();
    }

    @Override
    public void start() {
        follower.setStartingPose(startPose);

        triangle = follower.pathBuilder()
                .addPath(new BezierLine(startPose, interPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), interPose.getHeading())
                .addPath(new BezierLine(interPose, endPose))
                .setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
                .addPath(new BezierLine(endPose, startPose))
                .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
                .build();

        follower.followPath(triangle);
    }

    @Override
    public void loop() {
        follower.setPose(kalman.update()); // NEW

        follower.update();
        draw();

        if (follower.atParametricEnd()) {
            follower.followPath(triangle, true);
        }
    }
}
