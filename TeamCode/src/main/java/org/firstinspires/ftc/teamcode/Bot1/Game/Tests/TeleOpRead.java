/*********************
 * This is the testing code for read variables at the beginning of teleOp
 * NOT VERIFIED -- JUST FOR IDEA
 ***************************/

package org.firstinspires.ftc.teamcode.Bot1.Game.Tests;

// TeleOp (Iterative)

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Pedro.util.Tuning;
import org.firstinspires.ftc.teamcode.Pedro.util.TuningIO;
@Disabled
@TeleOp(name = "TeleOp: Load Tuning")
public class TeleOpRead extends OpMode {
    @Override public void init() {
        TuningIO.load();  // <- read tuning.json from app-owned storage
        telemetry.addData("kP", Tuning.kP);
        telemetry.addData("targetRpm", Tuning.targetRpm);
        telemetry.update();
    }

    @Override public void loop() {
        // use Tuning.kP, Tuning.targetRpm, etc.
    }

    @Override public void stop() {
        // Optional: persist any tweaks you made during TeleOp
        TuningIO.save();
    }
}

