/*********************
 * This is the testing code for save variables at the end of Auto
 * NOT VERIFIED -- JUST FOR IDEA
 ***************************/

package org.firstinspires.ftc.teamcode.Bot1.Game.Tests;

//import org.firstinspires.ftc.teamcode.Pedro.util.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pedro.util.TuningIO;


@Autonomous(name="Auto: Save Tuning at End")
@Disabled

public class AutoIterativeSave extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Load any previously saved values
        TuningIO.load();

        // TODO: init your hardware here

        waitForStart();

        boolean completed = false;
        try {
            if (isStopRequested()) return;

            // --- your autonomous logic ---
            // If you adjust variables during auto, update Tuning.* as you go:
            // Tuning.kP = newKp;
            // Tuning.targetRpm = measuredRpm;

            // ... do paths, actions, etc. ...

            completed = true;  // reached the end of auto successfully
        } finally {
            // Save no matter what; or save only if completed==true
            // (choose the one you prefer)
            // if (completed)
            TuningIO.save();

            telemetry.addLine("Tuning saved.");
            telemetry.update();
            sleep(200); // small pause so the write hits disk
        }
    }

    // Optional belt-and-suspenders: SDK calls stop() after op mode ends.
    /*
    @Override
    public void stop() {
        TuningIO.save();
        super.stop();
    }*/
}
