package org.firstinspires.ftc.teamcode.Pedro.util;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.json.JSONObject;
import java.io.File;

public final class TuningIO {
    private static final File FILE = AppUtil.getInstance().getSettingsFile("tuning.json");

    public static void save() {
        try {
            JSONObject o = new JSONObject();
            o.put("kP", Tuning.kP);
            o.put("kI", Tuning.kI);
            o.put("kD", Tuning.kD);
            o.put("targetRpm", Tuning.targetRpm);
            ReadWriteFile.writeFile(FILE, o.toString(2));
        } catch (Exception ignored) {}
    }

    public static void load() {
        try {
            if (!FILE.exists()) return;
            JSONObject o = new JSONObject(ReadWriteFile.readFile(FILE));
            Tuning.kP        = o.optDouble("kP", Tuning.kP);
            Tuning.kI        = o.optDouble("kI", Tuning.kI);
            Tuning.kD        = o.optDouble("kD", Tuning.kD);
            Tuning.targetRpm = o.optDouble("targetRpm", Tuning.targetRpm);
        } catch (Exception ignored) {}
    }

    private TuningIO() {}
}
