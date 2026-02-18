package org.firstinspires.ftc.teamcode.Bot1.Game.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Pedro.pedroPathing.Constants;

/**************************************
 * Manual driving through Pedro follower (robot-centric)
 * - Deadband + Expo shaping
 * - A: build path from CURRENT pose -> loadPose -> scorePose and follow it
 * - X: follow fixed scorePose -> loadPose -> scorePose chain
 * - Y: toggle SoftHold at scorePose
 * - Hold LT: manual override (cancels auto/hold and lets driver control)
 * - B: cancel auto/hold, return to manual
 * - Auto: when path finishes, optionally auto-hold at scorePose (if close enough)
 * - AutoStopHold: on stick release, do brake pulse then hold a (slightly lead) pose
 * - kS in hold comes from Pedro PIDF F; kS is faded in to reduce snap-back
 *****************************************/
//@Disabled
@TeleOp(name="Pedro: Brake", group="Tests")
public class PedroTeleOp_Brake extends LinearOpMode {

    private Follower follower;
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;

    private final String leftFrontMotorName  = "leftFront";
    public  String leftRearMotorName         = "leftBack";
    public  String rightFrontMotorName       = "rightFront";
    public  String rightRearMotorName        = "rightBack";

    // =========================
    // Target poses (inches, radians)
    // =========================
    private final Pose startPose = new Pose(62, 8, 0);
    private final Pose loadPose  = new Pose(25, 18, Math.toRadians(0));
    private final Pose scorePose = new Pose(59, 23, Math.toRadians(90));
    private final Pose parkPose = new Pose (35, 24, Math.toRadians(90));
    private final Pose nearShootPose = new Pose (49, 73, 135);
    private final Pose nearShootPose2 = new Pose (36, 86, 135);
    private final Pose nearShootPose3 = new Pose (23, 92, 135);
    // Fixed chain: score -> load -> score
    private PathChain scoreToLoadToScore;

    // =========================
    // Mode state machine
    // =========================
    private enum Mode { MANUAL, HOLD, AUTO, STOP_PATH }
    private Mode mode = Mode.MANUAL;

    private Pose holdPose = null;               // hold target pose
    private boolean autoStopHold = false;       // true only for stick-release holds (enables brake pulse)
    private boolean autoHoldAfterFinish = false;// latch: after A/X auto, try to hold at score if close
    private Pose stopHoldPose = null;   // where we will hold after STOP_PATH finishes
    private Pose autoHoldTarget = null; // which pose to hold after AUTO finishes (null = no hold)
    // =========================
    // Edge detection
    // =========================
    Gamepad currentGamepad1  = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    private boolean lastA = false, lastB = false, lastX = false, lastY = false,  lastL3 = false;
    private boolean lastDpadDown=false, lastDpadUp=false, lastDpadLeft=false, lastDpadRight=false;
    // =========================
    // Manual drive shaping
    // =========================
    private static final double DEAD_BAND  = 0.07;
    private static final double EXPO_K     = 0.55;  // 0..1 (0 linear, 1 cubic)
    private static final double TURN_SCALE = 0.75;

    // Hold-to-drive override
    private static final double LT_OVERRIDE_THRESH = 0.30;

    // =========================
    // Soft hold tuning
    // =========================
    private static final double HOLD_KP_XY = 0.06;
    private static final double HOLD_KP_H  = 0.90;

    private static final double HOLD_MAX_SPEED  = 0.30;
    private static final double HOLD_MAX_STRAFE = 0.30;
    private static final double HOLD_MAX_TURN   = 0.45;

    // Auto-hold-at-score entry tolerance
    private static final double HOLD_START_TOL_XY_IN = 3.0;
    private static final double HOLD_START_TOL_H_RAD = Math.toRadians(15);

    // Hold deadbands for enabling kS
    private static final double HOLD_ERR_DB_XY_IN = 0.25;
    private static final double HOLD_ERR_DB_H_RAD = Math.toRadians(3);

    // kS (from Pedro PIDF F terms) – kept as requested
    private static final double HOLD_KS_MOVE   = Constants.followerConstants.coefficientsDrivePIDF.F;
    private static final double HOLD_KS_STRAFE = Constants.followerConstants.coefficientsTranslationalPIDF.F;
    private static final double HOLD_KS_TURN   = Constants.followerConstants.coefficientsHeadingPIDF.F;

    // kS fade-in windows (inches / radians)
    private static final double KS_FADE_XY_IN = 10.0;
    private static final double KS_FADE_H_RAD = Math.toRadians(12);

    // =========================
    // AutoStopHold on stick release
    // =========================
    private boolean lastDriverActive = false;
    private int releaseStableLoops = 0;

    private static final int RELEASE_STABLE_LOOPS = 1;
    private static final double RELEASE_DB = 0.08;
    private static final double RELEASE_TURN_DB = 0.10;
    private static final double RELEASE_TURN_SETTLE_DB = 0.03; // small, to avoid heading impulse

    // Lead the hold target on release (helps reduce snap-back)
    private static final double RELEASE_LEAD_IN_MAX = 18.0;
    private static final double RELEASE_LEAD_POWER_EXP = 1.0;

    // Brake pulse
    private double lastCmdSpeed = 0, lastCmdStrafe = 0, lastCmdTurn = 0;
    private long brakePulseEndMs = 0;

    private static final int BRAKE_PULSE_MS = 120;
    private static final double BRAKE_GAIN = 0.5;

    private long holdStartMs = 0;
    private static final long HOLD_RAMP_MS = 200; // 150–300ms

    private long stopSettleEndMs = 0;
    private long stopCooldownEndMs = 0;

    private static final int STOP_SETTLE_MS = 150;    // 120–200ms typical
    private static final int STOP_COOLDOWN_MS = 250;  // prevents “stop-stop-stop” loops

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);

        leftFront  = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear   = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        rightRear  = hardwareMap.get(DcMotorEx.class, rightRearMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        scoreToLoadToScore = buildScoreToLoadToScore();
        follower.setStartingPose(startPose);

        follower.startTeleopDrive();
        mode = Mode.MANUAL;

        telemetry.addLine("A: Current→Load→Score | X: Score→Load→Score");
        telemetry.addLine("Y: Toggle SoftHold @ scorePose | B: Cancel");
        telemetry.addLine("Hold LT: Manual override (cancel auto/hold)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            follower.update();
            Pose currentPose = follower.getPose();

            // -------------------------
            // Button edge detection
            // -------------------------
            // Save previous state
            previousGamepad1.copy(currentGamepad1);
            // Read current state
            currentGamepad1.copy(gamepad1);
            boolean aPress  =  currentGamepad1.a && !previousGamepad1.a;
            boolean bPress  =  currentGamepad1.b && !previousGamepad1.b;
            boolean xPress  =  currentGamepad1.x && !previousGamepad1.x;
            boolean yPress  =  currentGamepad1.y && !previousGamepad1.y;

            boolean ddPress =  currentGamepad1.dpad_down  && !previousGamepad1.dpad_down;
            boolean duPress =  currentGamepad1.dpad_up    && !previousGamepad1.dpad_up;
            boolean dlPress =  currentGamepad1.dpad_left  && !previousGamepad1.dpad_left;
            boolean drPress =  currentGamepad1.dpad_right && !previousGamepad1.dpad_right;

            boolean l3Press =  currentGamepad1.left_stick_button &&
                    !previousGamepad1.left_stick_button;

            // -------------------------
            // LT override: immediate manual control
            // -------------------------
            if (gamepad1.left_trigger > LT_OVERRIDE_THRESH) {
                exitToManual();
                driveManualWithDeadbandExpo();
                telemetry.addData("mode", "MANUAL OVERRIDE (LT)");
                telemetry.addData("pose", "x=%.2f y=%.2f h=%.1f deg",
                        currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
                telemetry.update();
                continue;
            }

            // -------------------------
            // B: cancel everything
            // -------------------------
            if (bPress) {
                exitToManual();
            }

            // -------------------------
            // Y: toggle hold at scorePose (only when not in AUTO)
            // -------------------------
            if (yPress && mode != Mode.AUTO) {
                if (mode == Mode.HOLD) {
                    exitToManual();
                } else {
                    enterHold(scorePose, false); // false => not autoStopHold
                }
            }

            // =========================
            // MAIN BEHAVIOR (fixed)
            // =========================
            //1) A,or X or left stick press, then 3 auto path.
            if ((aPress || xPress || l3Press) && mode != Mode.AUTO) {
                exitToManual(); // cancels HOLD or STOP_PATH cleanly

                if (l3Press) {
                    startAutoPath(buildCurrentToPark(currentPose), null);
                    autoHoldAfterFinish = false;   // park ends in free manual
                } else if (aPress) {
                    startAutoPath(buildCurrentToLoadToScore(currentPose), scorePose);
                    // autoHoldAfterFinish remains true (set by startAutoPath)
                } else if (xPress) {
                    startAutoPath(scoreToLoadToScore, scorePose);
                    // autoHoldAfterFinish remains true (set by startAutoPath)
                }

                continue; // don't run other behaviors this loop
            }

            // ---- D-PAD preset shots: go to pose and HOLD ----
            if ((ddPress || drPress || duPress || dlPress) && mode != Mode.AUTO) {
                exitToManual(); // cancels HOLD or STOP_PATH cleanly

                Pose target = null;
                if (ddPress) target = scorePose;
                else if (drPress) target = nearShootPose;
                else if (duPress) target = nearShootPose2;
                else if (dlPress) target = nearShootPose3;

                if (target != null) {
                    startAutoPath(buildCurrentToTarget(currentPose, target), target); // HOLD at the target
                }

                continue; // IMPORTANT: don't run other behaviors this loop
            }

                // 2) AUTO: check completion
            if (mode == Mode.AUTO) {
                if (!follower.isBusy()) {
                    follower.startTeleopDrive();
                    mode = Mode.MANUAL;

                    if (autoHoldAfterFinish && autoHoldTarget != null) {
                        Pose p = follower.getPose();

                        double dxy = Math.hypot(autoHoldTarget.getX() - p.getX(),
                                autoHoldTarget.getY() - p.getY());
                        double dh  = Math.abs(wrapRad(autoHoldTarget.getHeading() - p.getHeading()));

                        if (dxy <= HOLD_START_TOL_XY_IN && dh <= HOLD_START_TOL_H_RAD) {
                            // Hold at the target heading; keep current x,y to avoid snap
                            enterHold(new Pose(p.getX(), p.getY(), autoHoldTarget.getHeading()), false);
                        } else {
                            exitToManual();
                        }
                    }

                    autoHoldAfterFinish = false;
                    autoHoldTarget = null;
                }
                // While AUTO is active, do nothing else; follower.update() drives it
            }

            // 3) HOLD
            else if (mode == Mode.HOLD && holdPose != null) {
                if (driverActive()) {
                    exitToManual();
                    driveManualWithDeadbandExpo();
                } else {
                    long nowMs = System.currentTimeMillis();
                    if (autoStopHold && nowMs < brakePulseEndMs) applyBrakePulse();
                    else softHoldToPose(holdPose);
                }
            }
/***********this is purely with pedro no other correction. Working mechanism
        // 4) STOP_PATH
            else if (mode == Mode.STOP_PATH) {
                if (driverActive()) {
                    exitToManual();
                    driveManualWithDeadbandExpo();
                } else if (!follower.isBusy()) {
                    exitToManual();
                    follower.setTeleOpDrive(0, 0, 0, true); // debug: no hold
                }
            }
*************************/
            // 4) STOP_PATH
            // /********TRY to MAKE IT STOP Smoothly**********/
            else if (mode == Mode.STOP_PATH) {

                // Any driver input cancels stop assist immediately
                if (driverActive()) {
                    exitToManual();
                    driveManualWithDeadbandExpo();
                    continue;
                }

                // If STOP_PATH finished, go to MANUAL but enforce a settle window
                if (!follower.isBusy()) {
                    exitToManual();

                    long now = System.currentTimeMillis();
                    stopSettleEndMs = now + STOP_SETTLE_MS;
                    stopCooldownEndMs = now + STOP_COOLDOWN_MS;

                    // Command 0 right now
                    follower.setTeleOpDrive(0, 0, 0, true);
                    continue;
                }

                // While STOP_PATH is running, do nothing else (follower.update() drives it)
                continue;
            }
            // 5) MANUAL
            else { // mode == MANUAL
                boolean active = driverActive();

                if (!active) releaseStableLoops++;
                else releaseStableLoops = 0;

                // If we just released sticks, engage stop assist
                if (lastDriverActive && releaseStableLoops >= RELEASE_STABLE_LOOPS) {
                    if (Math.abs(lastCmdTurn) < RELEASE_TURN_SETTLE_DB) {
                        startStopPathFromRelease();
                    } else {
                        driveManualWithDeadbandExpo();
                    }
                } else {
                    driveManualWithDeadbandExpo();
                }

                lastDriverActive = active; // IMPORTANT
            }


            // -------------------------
            // Telemetry
            // -------------------------
            telemetry.addData("mode", mode);
            telemetry.addData("sticks", "ly=%.2f lx=%.2f rx=%.2f",
                    gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            telemetry.addData("cmd", "v=%.2f s=%.2f t=%.2f", lastCmdSpeed, lastCmdStrafe, lastCmdTurn);

            telemetry.addData("lastCmdTurn", lastCmdTurn);
            telemetry.addData("brake br", -BRAKE_GAIN * lastCmdTurn);
            telemetry.addData("mode", mode);
            telemetry.addData("autoHoldAfterFinish", autoHoldAfterFinish);
            telemetry.addData("autoStopHold", autoStopHold);
            telemetry.addData("busy", follower.isBusy());
            telemetry.addData("pose", "x=%.2f y=%.2f h=%.1f deg",
                    currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
            telemetry.update();
        }
    }

    // =========================================================
    // Helpers: mode transitions
    // =========================================================
    private void exitToManual() {
        mode = Mode.MANUAL;
        stopHoldPose = null;
        holdPose = null;
        autoStopHold = false;
        autoHoldAfterFinish = false;

        releaseStableLoops = 0;
        lastDriverActive = false;
        brakePulseEndMs = 0;

        follower.startTeleopDrive();
    }

    private void enterHold(Pose target, boolean isAutoStopHold) {
        mode = Mode.HOLD;
        holdPose = target;
        autoStopHold = isAutoStopHold;
        holdStartMs = System.currentTimeMillis();
        follower.startTeleopDrive();
    }

    private void startAutoPath(PathChain chain, Pose holdTargetOrNull) {
        // Clear hold/brake state and start auto
        holdPose = null;
        autoStopHold = false;
        releaseStableLoops = 0;
        lastDriverActive = false;
        brakePulseEndMs = 0;

        follower.startTeleopDrive();
        follower.followPath(chain, true);

        mode = Mode.AUTO;

        autoHoldTarget = holdTargetOrNull;
        autoHoldAfterFinish = (autoHoldTarget != null);
    }

    private void applyBrakePulse() {
        double bs = clip(-BRAKE_GAIN * lastCmdSpeed,  -HOLD_MAX_SPEED,  HOLD_MAX_SPEED);
        double bt = clip(-BRAKE_GAIN * lastCmdStrafe, -HOLD_MAX_STRAFE, HOLD_MAX_STRAFE);

        // IMPORTANT: don't rotate during brake pulse (prevents spin on stick release)
       // double br = 0.0;
        // if it's turning on purpose, apply the counter-turn control
        /*double br = (Math.abs(lastCmdTurn) > 0.10)
                ? clip(-BRAKE_GAIN * lastCmdTurn, -HOLD_MAX_TURN, HOLD_MAX_TURN)
                : 0.0;*/
        double br = 0;

        follower.setTeleOpDrive(bs, bt, br, true);
    }


    private void enterAutoStopHoldWithLead() {
        Pose p = follower.getPose();
        double h = p.getHeading();

        double uF = lastCmdSpeed;
        double uS = lastCmdStrafe;

        double mag = clip(Math.hypot(uF, uS), 0.0, 1.0);
        double leadIn = RELEASE_LEAD_IN_MAX * Math.pow(mag, RELEASE_LEAD_POWER_EXP);

        double dirF = (mag > 1e-6) ? (uF / mag) : 0.0;
        double dirS = (mag > 1e-6) ? (uS / mag) : 0.0;

        // Robot-frame dir -> field-frame lead
        double leadX = ( Math.cos(h) * dirF - Math.sin(h) * dirS) * leadIn;
        double leadY = ( Math.sin(h) * dirF + Math.cos(h) * dirS) * leadIn;

        Pose target = new Pose(p.getX() + leadX, p.getY() + leadY, p.getHeading());

        brakePulseEndMs = System.currentTimeMillis() + BRAKE_PULSE_MS;
        enterHold(target, true);

        releaseStableLoops = 0;
        lastDriverActive = false;
    }

    // =========================================================
    // Manual drive: deadband + expo
    // =========================================================
    private void driveManualWithDeadbandExpo() {
        double speed  = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn   = -gamepad1.right_stick_x * TURN_SCALE;

        speed  = expo(deadband(speed,  DEAD_BAND), EXPO_K);
        strafe = expo(deadband(strafe, DEAD_BAND), EXPO_K);
        turn   = expo(deadband(turn,   DEAD_BAND), EXPO_K);

        lastCmdSpeed  = speed;
        lastCmdStrafe = strafe;
        lastCmdTurn   = turn;

        follower.setTeleOpDrive(speed, strafe, turn, true);
    }

    private static double deadband(double x, double db) {
        return (Math.abs(x) < db) ? 0.0 : x;
    }

    // Expo blend: k*x^3 + (1-k)*x
    private static double expo(double x, double k) {
        return k * x * x * x + (1.0 - k) * x;
    }

    // =========================================================
// Soft hold: resist pushes at a target pose (ramped-in, kS faded)
//  - Uses P on XY + heading
//  - Ramps translation in for the first HOLD_RAMP_MS to reduce snap-back after STOP_PATH ends
//  - kS comes from Pedro PIDF F terms, faded in based on error magnitude (prevents kick near target)
//  - Optional: if autoStopHold is true, we do NOT correct heading (prevents rotate-on-release)
// =========================================================
    private void softHoldToPose(Pose target) {
        Pose current = follower.getPose();

        // Field-frame error
        double ex = target.getX() - current.getX();
        double ey = target.getY() - current.getY();

        // Heading error wrapped to [-pi, pi]
        double eh = wrapRad(target.getHeading() - current.getHeading());

        // Convert field error -> robot-centric errors
        double h = current.getHeading();
        double forwardErr =  Math.cos(h) * ex + Math.sin(h) * ey;
        double strafeErr  = -Math.sin(h) * ex + Math.cos(h) * ey;

        // -------------------------
        // Base P corrections
        // -------------------------
        double speedCmd  = HOLD_KP_XY * forwardErr;
        double strafeCmd = HOLD_KP_XY * strafeErr;

        // Optional: suppress heading correction during AutoStopHold to prevent post-release rotation
        double turnCmd = autoStopHold ? 0.0 : (HOLD_KP_H * eh);

        // -------------------------
        // Ramp-in (reduces snap-back right after STOP_PATH finishes)
        // - translation ramps from 0 -> 1 over HOLD_RAMP_MS
        // - heading is NOT ramped (usually feels better), but you can if you want
        // -------------------------
        double ramp = 1.0;
        if (holdStartMs > 0) {
            long nowMs = System.currentTimeMillis();
            ramp = clip((nowMs - holdStartMs) / (double) HOLD_RAMP_MS, 0.0, 1.0);
        }
        speedCmd  *= ramp;
        strafeCmd *= ramp;

        // -------------------------
        // kS stiction compensation with fade-in (NO derivative, low noise)
        // Increase fade window (e.g., 10") to avoid "kick" near target
        // -------------------------

        double ksScaleFwd = clip(
                (Math.abs(forwardErr) - HOLD_ERR_DB_XY_IN) / KS_FADE_XY_IN,
                0.0, 1.0
        );

        double ksScaleStrafe = clip(
                (Math.abs(strafeErr) - HOLD_ERR_DB_XY_IN) / KS_FADE_XY_IN,
                0.0, 1.0
        );

        double ksScaleTurn = clip(
                (Math.abs(eh) - HOLD_ERR_DB_H_RAD) / KS_FADE_H_RAD,
                0.0, 1.0
        );

        if (Math.abs(forwardErr) > HOLD_ERR_DB_XY_IN && Math.abs(speedCmd) > 1e-6) {
            speedCmd += Math.signum(speedCmd) * HOLD_KS_MOVE * ksScaleFwd;
        }

        if (Math.abs(strafeErr) > HOLD_ERR_DB_XY_IN && Math.abs(strafeCmd) > 1e-6) {
            strafeCmd += Math.signum(strafeCmd) * HOLD_KS_STRAFE * ksScaleStrafe;
        }

        if (!autoStopHold && Math.abs(eh) > HOLD_ERR_DB_H_RAD && Math.abs(turnCmd) > 1e-6) {
            turnCmd += Math.signum(turnCmd) * HOLD_KS_TURN * ksScaleTurn;
        }

        // -------------------------
        // Clamp outputs to avoid oscillation / aggressive driving
        // -------------------------
        double speed  = clip(speedCmd,  -HOLD_MAX_SPEED,  HOLD_MAX_SPEED);
        double strafe = clip(strafeCmd, -HOLD_MAX_STRAFE, HOLD_MAX_STRAFE);
        double turn   = clip(turnCmd,   -HOLD_MAX_TURN,   HOLD_MAX_TURN);

        follower.setTeleOpDrive(speed, strafe, turn, true);

        // Optional telemetry (comment out if too chatty)
        telemetry.addData("hold err", "f=%.2f s=%.2f h=%.2fdeg", forwardErr, strafeErr, Math.toDegrees(eh));
        telemetry.addData("hold cmd", "v=%.2f s=%.2f t=%.2f ramp=%.2f", speed, strafe, turn, ramp);
    }


    // =========================================================
    // Driver activity
    // =========================================================
    private boolean driverActive() {
        return (Math.abs(gamepad1.left_stick_y) > RELEASE_DB) ||
                (Math.abs(gamepad1.left_stick_x) > RELEASE_DB) ||
                (Math.abs(gamepad1.right_stick_x) > RELEASE_TURN_DB);
    }

    // =========================================================
    // Utilities
    // =========================================================
    private static double wrapRad(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // =========================================================
    // Paths (BezierLine only + linear heading interpolation)
    // =========================================================
    private PathChain buildScoreToLoadToScore() {
        Path p1 = new Path(new BezierLine(scorePose, loadPose));
        p1.setLinearHeadingInterpolation(scorePose.getHeading(), loadPose.getHeading());

        Path p2 = new Path(new BezierLine(loadPose, scorePose));
        p2.setLinearHeadingInterpolation(loadPose.getHeading(), scorePose.getHeading());

        return follower.pathBuilder()
                .addPath(p1)
                .addPath(p2)
                .build();
    }

    private PathChain buildCurrentToLoadToScore(Pose currentPose) {
        Pose start = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading());

        Path p1 = new Path(new BezierLine(start, loadPose));
        p1.setLinearHeadingInterpolation(start.getHeading(), loadPose.getHeading());

        Path p2 = new Path(new BezierLine(loadPose, scorePose));
        p2.setLinearHeadingInterpolation(loadPose.getHeading(), scorePose.getHeading());

        return follower.pathBuilder()
                .addPath(p1)
                .addPath(p2)
                .build();
    }

    private PathChain buildCurrentToTarget(Pose currentPose, Pose targetPose) {
        Pose start = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading());

        Path p = new Path(new BezierLine(start, targetPose));
        p.setLinearHeadingInterpolation(start.getHeading(), targetPose.getHeading());

        return follower.pathBuilder()
                .addPath(p)
                .build();
    }
    private void startStopPathFromRelease() {
        Pose p = follower.getPose();
        double h = p.getHeading();

        // lastCmdSpeed/Strafe are robot-centric commands in [-1..1]
        double uF = lastCmdSpeed;
        double uS = lastCmdStrafe;

        double mag = clip(Math.hypot(uF, uS), 0.0, 1.0);

        lastCmdTurn = 0.0;   // prevents any leftover turn history from influencing your logic/telemetry

        // Use a SHORT lead for a "stop assist" (recommended 2-12")
        double leadIn = 3.0 + 5.0 * Math.pow(mag, 1.0);

        // Direction in robot frame
        double dirF = (mag > 1e-6) ? (uF / mag) : 0.0;
        double dirS = (mag > 1e-6) ? (uS / mag) : 0.0;

        // Robot-frame direction -> field-frame lead
        double leadX = ( Math.cos(h) * dirF - Math.sin(h) * dirS) * leadIn;
        double leadY = ( Math.sin(h) * dirF + Math.cos(h) * dirS) * leadIn;

        Pose leadPose = new Pose(p.getX() + leadX, p.getY() + leadY, p.getHeading());

        // Build a tiny path p -> leadPose (BezierLine)
        Path pStop = new Path(new BezierLine(p, leadPose));
        pStop.setConstantHeadingInterpolation(p.getHeading());

        PathChain stopChain = follower.pathBuilder()
                .addPath(pStop)
                .build();

        // Clear any hold state and start path
        holdPose = null;
        autoStopHold = false;
        releaseStableLoops = 0;
        lastDriverActive = false;
        brakePulseEndMs = 0;

        follower.startTeleopDrive();
        follower.followPath(stopChain, true);

        // After STOP_PATH ends, we will softHold at this leadPose
        stopHoldPose = leadPose;
        mode = Mode.STOP_PATH;
    }

    private PathChain buildCurrentToPark(Pose currentPose) {
        Pose start = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading());

        Path p1 = new Path(new BezierLine(start, parkPose));
        p1.setLinearHeadingInterpolation(start.getHeading(), parkPose.getHeading());

        return follower.pathBuilder()
                .addPath(p1)
                .build();
    }

}