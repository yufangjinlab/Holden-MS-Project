package org.firstinspires.ftc.teamcode.Bot1.Game.Tests;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Pedro.pedroPathing.Constants;

//@Disabled
@TeleOp(name = "Pedro Direction 4Direction", group = "Test")
public class PedroDirectionNewTest extends OpMode {

    private Follower follower;

    // Edge-detection gamepad
    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();

    // Distance and angle constants
    private static final double DISTANCE_IN = 40.0;      // Distance to move for each test (inches)
    private static final double ANGLE_DEG   = 45.0;      // Base angle (degrees)

    // Direction-dependent distance compensation from 1s tests
    private static final double FWD_FORWARD_SCALE  = 1.0;//1.023;  // forward moves slightly longer
    private static final double FWD_BACK_SCALE     = 1;//0.978;  // backward moves slightly shorter

    private static final double STRAFE_LEFT_SCALE  = 1;//0.980;  // robot-left moves slightly shorter
    private static final double STRAFE_RIGHT_SCALE = 1;  // robot-right moves slightly longer

    private static final double STRAFE_EPS         = 1e-3;

    // --- Motion metrics ---
    private ElapsedTime motionTimer = new ElapsedTime();

    // For instantaneous & max velocity
    private Pose lastPose = null;
    private double lastTime = 0.0;
    private double maxVel = 0.0;

    // Latest velocity components (for logging / telemetry)  // NEW
    private double vxField = 0.0;   // in/s (field frame)
    private double vyField = 0.0;   // in/s (field frame)
    private double vxRobot = 0.0;   // in/s (robot frame, +forward)
    private double vyRobot = 0.0;   // in/s (robot frame, +left)
    private double instSpeed = 0.0; // in/s

    // For mid-velocity sample (between 0.3s and 0.7s)
    private boolean midSampleCaptured = false;
    private double midVel = 0.0;
    private double midDx = 0.0;
    private double midDy = 0.0;
    private double midDt = 0.0;

    // For detecting "just finished" to print one-time results
    private boolean lastBusy = false;

    // Warm-up detection for Pinpoint
    private boolean pinpointWarm = false;
    private ElapsedTime warmupTimer = new ElapsedTime();
    private double lastHeadingRad = 0.0;
    private double maxHeadingDriftDeg = 0.0;

    // How long to watch for drift, and what drift is acceptable
    private static final double WARMUP_DURATION_SEC = 1.0;    // 1 second
    private static final double WARMUP_MAX_DRIFT_DEG = 0.2;   // allow up to 0.2°

    private ElapsedTime globalTimer = new ElapsedTime();

    @Override
    public void init() {
        // Your Constants.createFollower(hardwareMap) from Pedro setup
        follower = Constants.createFollower(hardwareMap);

        // ---- Reset Pinpoint for clean run ----
        try {
            GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.resetPosAndIMU();
        } catch (Exception e) {
            telemetry.addLine("Pinpoint reset failed — check config name");
        }
        globalTimer.reset();
        // Set starting pose (change if you want to use field coords)
        follower.setStartingPose(new Pose(0, 0, 0));   // (x,y,heading) in Pedro coords
        follower.update();

        // Warm-up init
        pinpointWarm = false;
        warmupTimer.reset();
        lastHeadingRad = follower.getPose().getHeading();
        maxHeadingDriftDeg = 0.0;

        telemetry.addLine("DirectionTest with Pinpoint warm-up");

        telemetry.addLine("Pedro Direction Test");
        telemetry.addLine("Dpad: up/down/left/right = +X/-X/+Y/-Y");
        telemetry.addLine("B/A/Y/X = diagonal moves with ±30°, 150°, 210°");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update follower each loop
        follower.update();
        boolean busy = follower.isBusy();
        double tGlobal = globalTimer.seconds();
        double tLocal  = motionTimer.seconds();  // keep this for mid-velocity
        //--- PINPOINT WARM-UP CHECK ---
        if (!pinpointWarm) {
            Pose p = follower.getPose();
            double currentHeadingRad = p.getHeading();

            // normalize delta heading to (-pi, pi)
            double dHeadingRad = AngleUnit.normalizeRadians(currentHeadingRad - lastHeadingRad);
            double dHeadingDeg = Math.toDegrees(dHeadingRad);
            double absDriftDeg = Math.abs(dHeadingDeg);

            if (absDriftDeg > maxHeadingDriftDeg) {
                maxHeadingDriftDeg = absDriftDeg;
            }

            lastHeadingRad = currentHeadingRad;

            // After WARMUP_DURATION_SEC, decide if drift is acceptable
            if (warmupTimer.seconds() >= WARMUP_DURATION_SEC) {
                if (maxHeadingDriftDeg <= WARMUP_MAX_DRIFT_DEG) {
                    pinpointWarm = true;
                } else {
                    // Restart measurement window if drift too large
                    warmupTimer.reset();
                    maxHeadingDriftDeg = 0.0;
                }
            }

            // Telemetry while warming up
            telemetry.addLine("Pinpoint warm-up: KEEP ROBOT STILL");
            telemetry.addData("Warm", pinpointWarm);
            telemetry.addData("Timer (s)", "%.2f", warmupTimer.seconds());
            telemetry.addData("Max heading drift (deg)", "%.3f", maxHeadingDriftDeg);
            telemetry.update();
            return; // <-- IMPORTANT: don't drive robot until warm
        }

        // Time since motion start (reset in resetMotionMetrics)   // NEW
        double t = motionTimer.seconds();

        if (busy) {
            Pose pNow = follower.getPose();
            if (pNow != null && lastPose != null) {
                double dt = t - lastTime;
                if (dt > 0.0) {
                    double dx = pNow.getX() - lastPose.getX();
                    double dy = pNow.getY() - lastPose.getY();

                    // --- Field-frame velocity (what you had before) ---  // NEW
                    vxField = dx / dt;   // in/s
                    vyField = dy / dt;   // in/s
                    instSpeed = Math.hypot(vxField, vyField);

                    // --- Robot-frame velocity (project world v into robot frame) ---  // NEW
                    double headingRad = pNow.getHeading();  // radians
                    vxRobot =  vxField * Math.cos(headingRad) + vyField * Math.sin(headingRad);
                    vyRobot = -vxField * Math.sin(headingRad) + vyField * Math.cos(headingRad);

                    // track max velocity (use speed magnitude)
                    if (instSpeed > maxVel) {
                        maxVel = instSpeed;
                    }

                    // capture one "mid" sample between 0.3s and 0.7s
                    if (!midSampleCaptured && t >= 0.3 && t <= 0.7) {
                        midSampleCaptured = true;
                        midVel = instSpeed;
                        midDx = dx;
                        midDy = dy;
                        midDt = dt;
                    }
                }

                lastPose = pNow;
                lastTime = t;
            }
        }

        // Edge detection for gamepad buttons
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        boolean dpadUpPressed    = currentGamepad1.dpad_up    && !previousGamepad1.dpad_up;
        boolean dpadDownPressed  = currentGamepad1.dpad_down  && !previousGamepad1.dpad_down;
        boolean dpadLeftPressed  = currentGamepad1.dpad_left  && !previousGamepad1.dpad_left;
        boolean dpadRightPressed = currentGamepad1.dpad_right && !previousGamepad1.dpad_right;

        boolean bPressed = currentGamepad1.b && !previousGamepad1.b;
        boolean aPressed = currentGamepad1.a && !previousGamepad1.a;
        boolean yPressed = currentGamepad1.y && !previousGamepad1.y;
        boolean xPressed = currentGamepad1.x && !previousGamepad1.x;

        // Don't start a new move if follower is busy
        if (!follower.isBusy()) {

            if (dpadUpPressed) {
                resetMotionMetrics();
                startMoveRobotRelative(+DISTANCE_IN, 0.0);   // forward
            } else if (dpadDownPressed) {
                resetMotionMetrics();
                startMoveRobotRelative(-DISTANCE_IN, 0.0);   // backward
            } else if (dpadLeftPressed) {
                resetMotionMetrics();
                startMoveRobotRelative(0.0, +DISTANCE_IN);   // left
            } else if (dpadRightPressed) {
                resetMotionMetrics();
                startMoveRobotRelative(0.0, -DISTANCE_IN);   // right
            }

            // Angled moves that also change heading:
            else if (aPressed) {
                resetMotionMetrics();
                startMovePolar(DISTANCE_IN, ANGLE_DEG);            // 30°
            } else if (bPressed) {
                resetMotionMetrics();
                startMovePolar(DISTANCE_IN, -ANGLE_DEG);           // -30°
            } else if (yPressed) {
                resetMotionMetrics();
                startMovePolar(-DISTANCE_IN, ANGLE_DEG);
            } else if (xPressed) {
                resetMotionMetrics();
                startMovePolar(-DISTANCE_IN, -ANGLE_DEG);
            }
        }

        // ---------------- TELEMETRY ------------------
        telemetry.addData("Follower busy", busy);

        Pose p = follower.getPose();
        if (p != null) {
            telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f",
                    p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        }

        // Detect the moment a motion just finished
        if (!busy && lastBusy) {
            telemetry.addLine("=== Motion Finished ===");
        }

        // Velocity metrics
        telemetry.addData("Max vel (in/s)", "%.2f", maxVel);

        telemetry.addData("v_field (in/s)", "vx=%.2f vy=%.2f", vxField, vyField);   // NEW
        telemetry.addData("v_robot (in/s)", "vx=%.2f vy=%.2f", vxRobot, vyRobot);   // NEW
        telemetry.addData("Speed (in/s)", "%.2f", instSpeed);                       // NEW

        if (midSampleCaptured) {
            telemetry.addData("Mid vel (in/s)", "%.2f", midVel);
            telemetry.addData("Mid dx,dy,dt",
                    "dx=%.4f dy=%.4f dt=%.4f", midDx, midDy, midDt);
        } else {
            telemetry.addLine("Mid vel: Not captured yet");
        }

        lastBusy = busy;

        // FINAL update
        telemetry.update();

        // -------- LOGCAT JSON (once per loop) --------  // NEW
        if (p != null) {
            double headingDeg = Math.toDegrees(p.getHeading());
            Log.i("PEDRO_DIR", String.format(
                    "{\"t\":%.3f,\"x\":%.3f,\"y\":%.3f,\"h_deg\":%.2f,"
                            + "\"vx_field\":%.3f,\"vy_field\":%.3f,"
                            + "\"vx_robot\":%.3f,\"vy_robot\":%.3f,"
                            + "\"speed\":%.3f,\"busy\":%b}",
                    tGlobal,
                    p.getX(), p.getY(), headingDeg,
                    vxField, vyField,
                    vxRobot, vyRobot,
                    instSpeed, busy
            ));
        }
    }



    /**
     * Move a given distance in the robot's local frame, without changing heading.
     *
     * @param forwardIn  + = forward, - = backward (inches)
     * @param strafeIn   + = left, - = right (inches)
     *  adjusted forawrdIn for backward motion such that there is less overshoot
     */
    /**
     * Move a given distance in the robot's local frame, without changing heading.
     *
     * @param forwardIn  + = forward, - = backward (inches)
     * @param strafeIn   + = left, - = right (inches)
     *  Direction-dependent compensation using today's measurements.
     */
    private void startMoveRobotRelative(double forwardIn, double strafeIn) {
        Pose start = follower.getPose();
        if (start == null) return;

        double h = start.getHeading(); // radians

        // --- apply direction-dependent compensation in ROBOT frame ---

        double f = forwardIn;
        double s = strafeIn;

        // Forward axis:
        //   f > 0 -> true forward
        //   f < 0 -> backward
        if (f > 0.0) {
            f *= FWD_FORWARD_SCALE;   // ~1.023
        } else if (f < 0.0) {
            f *= FWD_BACK_SCALE;      // ~0.978
        }

        // Lateral axis:
        //   s > 0 -> robot-left
        //   s < 0 -> robot-right
        if (Math.abs(s) > STRAFE_EPS) {
            if (s > 0.0) {
                // left strafe
                s *= STRAFE_LEFT_SCALE;   // ~0.980
            } else {
                // right strafe
                s *= STRAFE_RIGHT_SCALE;  // ~1.020
            }
        }

        // --- robot -> field frame using compensated distances ---
        double dxField = f * Math.cos(h) - s * Math.sin(h);
        double dyField = f * Math.sin(h) + s * Math.cos(h);

        Pose target = new Pose(
                start.getX() + dxField,
                start.getY() + dyField,
                h
        );

        BezierLine line = new BezierLine(follower::getPose, target);

        PathChain chain = follower.pathBuilder()
                .addPath(line)
                .setLinearHeadingInterpolation(h, h)
                .build();

        follower.followPath(chain);
    }

    private void startMovePolar(double distanceIn, double angleDeg) {
        Pose start = follower.getPose();
        if (start == null) return;

        double theta = Math.toRadians(angleDeg);
        double dx = distanceIn * Math.cos(theta);
        double dy = distanceIn * Math.sin(theta);

        Pose target = new Pose(
                start.getX() + dx,
                start.getY() + dy,
                theta    // heading becomes angleDeg
        );

        Path path = new Path(new BezierLine(follower::getPose, target));
        PathChain chain = follower.pathBuilder()
                .addPath(path)
                .build();

        follower.followPath(chain);
    }

    private void resetMotionMetrics() {
        motionTimer.reset();
        maxVel = 0.0;
        midVel = 0.0;
        midDx = 0.0;
        midDy = 0.0;
        midDt = 0.0;
        midSampleCaptured = false;

        lastPose = follower.getPose();
        lastTime = 0.0;

        // Reset instantaneous velocity state too      // NEW
        vxField = 0.0;
        vyField = 0.0;
        vxRobot = 0.0;
        vyRobot = 0.0;
        instSpeed = 0.0;
    }

    private void startMovePureTrans(double distanceIn, double angleDeg) {
        // angleDeg measured from +X axis, CCW positive
        Pose start = follower.getPose();
        if (start == null) return;

        double theta = Math.toRadians(angleDeg);
        double dx = distanceIn * Math.cos(theta);
        double dy = distanceIn * Math.sin(theta);

        // Raw (ideal) target in FIELD frame, heading forced to 0 rad
        Pose rawTarget = new Pose(
                start.getX() + dx,
                start.getY() + dy,
                0.0  // keep heading = 0°
        );

        // ✅ Compensated target using your direction-dependent scales
        Pose compTarget = compensateTargetPose(start, rawTarget);

        // Use the compensated pose as the final target
        follower.followPath(
                follower.pathBuilder()
                        .addPath(new BezierLine(start, compTarget))
                        .build()
        );
    }


    private Pose compensateTargetPose(Pose start, Pose rawTarget) {
        double h = start.getHeading(); // radians

        // --- field delta from start to raw target ---
        double dxFieldRaw = rawTarget.getX() - start.getX();
        double dyFieldRaw = rawTarget.getY() - start.getY();

        // --- field -> robot frame (forward, strafe) ---
        // forward: +X along robot's facing direction
        // strafe:  +Y to robot-left
        double forward =  dxFieldRaw * Math.cos(h) + dyFieldRaw * Math.sin(h);
        double strafe  = -dxFieldRaw * Math.sin(h) + dyFieldRaw * Math.cos(h);

        // --- apply direction-dependent compensation ---

        // Forward axis:
        //   forward > 0 -> "true forward" move
        //   forward < 0 -> backward move
        if (forward > 0.0) {
            forward *= FWD_FORWARD_SCALE;
        } else if (forward < 0.0) {
            forward *= FWD_BACK_SCALE;
        }

        // Lateral axis:
        //   strafe > 0 -> robot-left
        //   strafe < 0 -> robot-right
        if (Math.abs(strafe) > STRAFE_EPS) {
            if (strafe > 0.0) {
                // left strafe
                strafe /= STRAFE_LEFT_SCALE;
            } else {
                // right strafe
                strafe /= STRAFE_RIGHT_SCALE;
            }
        }

        // --- robot -> field frame (back to dx, dy in field) ---
        double dxFieldComp = forward * Math.cos(h) - strafe * Math.sin(h);
        double dyFieldComp = forward * Math.sin(h) + strafe * Math.cos(h);

        // Keep the rawTarget heading exactly (no extra rotation caused by compensation)
        return new Pose(
                start.getX() + dxFieldComp,
                start.getY() + dyFieldComp,
                rawTarget.getHeading()
        );
    }

}


