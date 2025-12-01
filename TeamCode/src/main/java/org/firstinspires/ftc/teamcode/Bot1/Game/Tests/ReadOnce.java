package org.firstinspires.ftc.teamcode.Bot1.Game.Tests;

import org.firstinspires.ftc.teamcode.Pedro.util.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.json.JSONObject;

import java.io.File;
@Disabled
@TeleOp(name="TeleOp: Inline Load")
public class ReadOnce extends OpMode {
    @Override public void init() {
        try {
            File file = AppUtil.getInstance().getSettingsFile("tuning.json");
            if (file.exists()) {
                String text = ReadWriteFile.readFile(file);
                JSONObject o = new JSONObject(text);
                // apply to your static vars or fields
                Tuning.kP        = o.optDouble("kP",        Tuning.kP);
                Tuning.kI        = o.optDouble("kI",        Tuning.kI);
                Tuning.kD        = o.optDouble("kD",        Tuning.kD);
                Tuning.targetRpm = o.optDouble("targetRpm", Tuning.targetRpm);
            }
            telemetry.addData("Loaded from", file.getAbsolutePath());
        } catch (Exception e) {
            telemetry.addData("Load error", e.getMessage());
        }
        telemetry.update();
    }

    @Override public void loop() { /* drive code using Tuning.* */ }
}

