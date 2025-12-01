package org.firstinspires.ftc.teamcode.Bot1.Game.TeleOp;

public enum Bot1FiniteState {
    IDLE,
    SPIN,
    LAUNCH_START,
    LAUNCHING,
    REVERSE_FEEDER,
    STOP_FEEDER,
    /************ Finite State for Auto  */
    DRIVING_TO_LAUNCH_POSE,
    FINAL_PARK,
    WAIT_DRIVE,
    DRIVING_AWAY_FROM_LAUNCH,
    COMPLETE;
}