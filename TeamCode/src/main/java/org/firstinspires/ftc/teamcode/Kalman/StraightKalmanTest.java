package org.firstinspires.ftc.teamcode.Kalman;

import static org.firstinspires.ftc.teamcode.Pedro.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.Pedro.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.Pedro.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.Pedro.pedroPathing.Tuning.telemetryM;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Pedro.pedroPathing.Constants;

@Autonomous(name = "Straight Kalman Test")
public class StraightKalmanTest extends OpMode {

    @IgnoreConfigurable
    public static TelemetryManager telemetryM;

    private KalmanTest kalman;

    private static final double DISTANCE = 30.0;

    private final Pose startPose = new Pose(0, 0, 0);
    private final Pose endPose   = new Pose(DISTANCE, 0, 0);

    private Path forwardPath;
    private Path backwardPath;

    private boolean goingForward = true;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        kalman = new KalmanTest();
    }

    @Override
    public void init_loop() {
        telemetryM.debug("Straight line Kalman sanity test");
        telemetryM.debug("Robot will move ±" + DISTANCE + " inches on X");
        telemetryM.update(telemetry);

        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        follower.setStartingPose(startPose);

        forwardPath = new Path(new BezierLine(startPose, endPose));
        forwardPath.setConstantHeadingInterpolation(0);

        backwardPath = new Path(new BezierLine(endPose, startPose));
        backwardPath.setConstantHeadingInterpolation(0);

        follower.followPath(forwardPath);
    }

    @Override
    public void loop() {
        follower.update();

        // === Raw pose ===
        Pose rawPose = follower.getPose();

        // === Kalman update ===
        Pose filteredPose = kalman.update(rawPose);
        //follower.setPose(filteredPose);

        draw();

        // === Path switching ===
        if (!follower.isBusy()) {
            if (goingForward) {
                follower.followPath(backwardPath);
            } else {
                follower.followPath(forwardPath);
            }
            goingForward = !goingForward;
        }

        // === Telemetry ===
        telemetryM.debug("Direction: " + (goingForward ? "FORWARD" : "BACKWARD"));
        telemetryM.debug("RAW:  x=" + rawPose.getX()
                + " y=" + rawPose.getY()
                + " h=" + rawPose.getHeading());
        telemetryM.debug("KF :  x=" + filteredPose.getX()
                + " y=" + filteredPose.getY()
                + " h=" + filteredPose.getHeading());
        telemetryM.update(telemetry);
    }
}
