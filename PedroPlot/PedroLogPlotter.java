import com.google.gson.Gson;
import com.google.gson.JsonObject;
import org.knowm.xchart.SwingWrapper;
import org.knowm.xchart.XYChart;
import org.knowm.xchart.XYChartBuilder;
import org.knowm.xchart.style.markers.None;
import org.knowm.xchart.BitmapEncoder;
import org.knowm.xchart.BitmapEncoder.BitmapFormat;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class PedroLogPlotter {

    private static final String LOG_FILE = "FB90.txt";

    // thresholds
    private static final double SPEED_MIN      = 0.5; // in/s: ignore almost-stationary samples
    private static final double VX_SIGN_THRESH = 0.5; // in/s: sign of vx_robot
    private static final double V_DROP_THRESH  = 1.0; // in/s: speed drop that counts as braking

    // JSON field names (adjust if your log uses different ones)
    private static final String FIELD_TIME   = "t";
    private static final String FIELD_X      = "x";
    private static final String FIELD_HEADING = "h_deg";
    private static final String FIELD_VX_ROBOT = "vx_robot";
    private static final String FIELD_VY_ROBOT = "vy_robot";
    private static final String FIELD_SPEED    = "speed";

    // ========================== MAIN ==============================
    public static void main(String[] args) throws IOException {

        // ---------- 1) Read JSON lines ----------
        List<Double> tList  = new ArrayList<>();
        List<Double> xList  = new ArrayList<>();
        List<Double> hList  = new ArrayList<>();
        List<Double> vxRList = new ArrayList<>();
        List<Double> vyRList = new ArrayList<>();
        List<Double> speedList = new ArrayList<>();

        Gson gson = new Gson();

        try (BufferedReader br = new BufferedReader(new FileReader(LOG_FILE))) {
            String line;
            while ((line = br.readLine()) != null) {
                line = line.trim();
                if (!line.startsWith("{")) continue;

                try {
                    JsonObject obj = gson.fromJson(line, JsonObject.class);
                    if (obj == null) continue;
                    if (!obj.has(FIELD_TIME)   ||
                        !obj.has(FIELD_X)      ||
                        !obj.has(FIELD_HEADING)||
                        !obj.has(FIELD_VX_ROBOT) ||
                        !obj.has(FIELD_VY_ROBOT) ||
                        !obj.has(FIELD_SPEED)) {
                        continue;
                    }

                    double tVal   = obj.get(FIELD_TIME).getAsDouble();
                    double xVal   = obj.get(FIELD_X).getAsDouble();
                    double hVal   = obj.get(FIELD_HEADING).getAsDouble();
                    double vxRVal = obj.get(FIELD_VX_ROBOT).getAsDouble();
                    double vyRVal = obj.get(FIELD_VY_ROBOT).getAsDouble();
                    double sVal   = obj.get(FIELD_SPEED).getAsDouble();

                    tList.add(tVal);
                    xList.add(xVal);
                    hList.add(hVal);
                    vxRList.add(vxRVal);
                    vyRList.add(vyRVal);
                    speedList.add(sVal);

                } catch (Exception e) {
                    // skip malformed JSON
                }
            }
        }

        if (tList.isEmpty()) {
            System.err.println("No valid data in " + LOG_FILE);
            return;
        }

        int n = tList.size();

        double[] t   = new double[n];
        double[] x   = new double[n];
        double[] hdg = new double[n];
        double[] vxR = new double[n];
        double[] vyR = new double[n];
        double[] sp  = new double[n];

        for (int i = 0; i < n; i++) {
            t[i]   = tList.get(i);
            x[i]   = xList.get(i);
            hdg[i] = hList.get(i);
            vxR[i] = vxRList.get(i);
            vyR[i] = vyRList.get(i);
            sp[i]  = speedList.get(i);
        }

        // ---------- 2) Sort all samples by time ----------
        Integer[] idx = new Integer[n];
        for (int i = 0; i < n; i++) idx[i] = i;

        final double[] tRef = t;
        Arrays.sort(idx, new Comparator<Integer>() {
            @Override
            public int compare(Integer a, Integer b) {
                return Double.compare(tRef[a], tRef[b]);
            }
        });

        double[] tS   = new double[n];
        double[] xS   = new double[n];
        double[] hdgS = new double[n];
        double[] vxRS = new double[n];
        double[] vyRS = new double[n];
        double[] spS  = new double[n];

        for (int k = 0; k < n; k++) {
            int i = idx[k];
            tS[k]   = t[i];
            xS[k]   = x[i];
            hdgS[k] = hdg[i];
            vxRS[k] = vxR[i];
            vyRS[k] = vyR[i];
            spS[k]  = sp[i];
        }

        t   = tS;
        x   = xS;
        hdg = hdgS;
        vxR = vxRS;
        vyR = vyRS;
        sp  = spS;

        // ---------- 3) Split into backward and forward segments ----------
        List<Double> backT   = new ArrayList<>();
        List<Double> backX   = new ArrayList<>();
        List<Double> backH   = new ArrayList<>();
        List<Double> backVxR = new ArrayList<>();
        List<Double> backVyR = new ArrayList<>();
        List<Double> backSp  = new ArrayList<>();

        List<Double> fwdT   = new ArrayList<>();
        List<Double> fwdX   = new ArrayList<>();
        List<Double> fwdH   = new ArrayList<>();
        List<Double> fwdVxR = new ArrayList<>();
        List<Double> fwdVyR = new ArrayList<>();
        List<Double> fwdSp  = new ArrayList<>();

        for (int i = 0; i < n; i++) {
            if (sp[i] < SPEED_MIN) continue;  // ignore very slow

            if (vxR[i] < -VX_SIGN_THRESH) {
                backT.add(t[i]);
                backX.add(x[i]);
                backH.add(hdg[i]);
                backVxR.add(vxR[i]);
                backVyR.add(vyR[i]);
                backSp.add(sp[i]);
            } else if (vxR[i] > VX_SIGN_THRESH) {
                fwdT.add(t[i]);
                fwdX.add(x[i]);
                fwdH.add(hdg[i]);
                fwdVxR.add(vxR[i]);
                fwdVyR.add(vyR[i]);
                fwdSp.add(sp[i]);
            }
        }

        MotionSegment backSeg = buildSegment(backT, backX, backH, backVxR, backVyR, backSp);
        MotionSegment fwdSeg  = buildSegment(fwdT,  fwdX,  fwdH,  fwdVxR,  fwdVyR,  fwdSp);

        if ((backSeg == null || backSeg.t.length < 2) &&
            (fwdSeg  == null || fwdSeg.t.length  < 2)) {
            System.err.println("No usable motion segments.");
            return;
        }

        // ---------- 4) Create plots ----------
       plotQuantity("Distance (in)", "FB90_Distance",
        backSeg == null ? null : backSeg.d,
        fwdSeg  == null ? null : fwdSeg.d,
        backSeg, fwdSeg);

plotQuantity("Heading (deg)", "FB90_Heading",
        backSeg == null ? null : backSeg.heading,
        fwdSeg  == null ? null : fwdSeg.heading,
        backSeg, fwdSeg);

plotQuantity("Speed (in/s)", "FB90_Speed",
        backSeg == null ? null : backSeg.speed,
        fwdSeg  == null ? null : fwdSeg.speed,
        backSeg, fwdSeg);

plotQuantity("vx_robot (in/s)", "FB90_Vx",
        backSeg == null ? null : backSeg.vx,
        fwdSeg  == null ? null : fwdSeg.vx,
        backSeg, fwdSeg);

plotQuantity("vy_robot (in/s)", "FB90_Vy",
        backSeg == null ? null : backSeg.vy,
        fwdSeg  == null ? null : fwdSeg.vy,
        backSeg, fwdSeg);

    }

    // ===================== HELPER CLASSES =========================

    private static class MotionSegment {
        double[] t;
        double[] d;
        double[] heading;
        double[] vx;
        double[] vy;
        double[] speed;
        Double brakeTime;  // local time
    }

    private static MotionSegment buildSegment(List<Double> tList,
                                              List<Double> xList,
                                              List<Double> hList,
                                              List<Double> vxList,
                                              List<Double> vyList,
                                              List<Double> sList) {
        if (tList == null || tList.size() < 2) return null;
        int n = tList.size();

        MotionSegment seg = new MotionSegment();
        seg.t       = new double[n];
        seg.d       = new double[n];
        seg.heading = new double[n];
        seg.vx      = new double[n];
        seg.vy      = new double[n];
        seg.speed   = new double[n];

        double t0 = tList.get(0);
        double x0 = xList.get(0);

        for (int i = 0; i < n; i++) {
            seg.t[i]       = tList.get(i) - t0;
            seg.d[i]       = xList.get(i) - x0;
            seg.heading[i] = hList.get(i);
            seg.vx[i]      = vxList.get(i);
            seg.vy[i]      = vyList.get(i);
            seg.speed[i]   = sList.get(i);
        }

        // braking: first significant speed drop after peak speed
        int peakIdx = 0;
        double peakS = seg.speed[0];
        for (int i = 1; i < n; i++) {
            if (seg.speed[i] > peakS) {
                peakS = seg.speed[i];
                peakIdx = i;
            }
        }

        Integer bIdx = null;
        for (int i = peakIdx + 1; i < n; i++) {
            if (seg.speed[i] < seg.speed[i - 1] - V_DROP_THRESH) {
                bIdx = i;
                break;
            }
        }

        if (bIdx != null) {
            seg.brakeTime = seg.t[bIdx];
        } else {
            seg.brakeTime = null;
        }

        return seg;
    }

    // ======================== PLOTTING =============================

    private static void plotQuantity(String yLabel,
                                     String fileName,
                                     double[] backY,
                                     double[] fwdY,
                                     MotionSegment backSeg,
                                     MotionSegment fwdSeg) {

        XYChart chart = new XYChartBuilder()
                .width(900)
                .height(600)
                .title(yLabel + " vs Time")
                .xAxisTitle("Time (s)")
                .yAxisTitle(yLabel)
                .build();

        double globalYMin = 0.0;
        double globalYMax = 0.0;
        boolean yInit = false;
        double tMax = 0.0;

        if (backSeg != null && backSeg.t.length > 1 && backY != null) {
            chart.addSeries("backward", backSeg.t, backY).setMarker(new None());
            double minB = Arrays.stream(backY).min().orElse(0.0);
            double maxB = Arrays.stream(backY).max().orElse(0.0);
            globalYMin = minB;
            globalYMax = maxB;
            yInit = true;
            tMax = backSeg.t[backSeg.t.length - 1];
        }

        if (fwdSeg != null && fwdSeg.t.length > 1 && fwdY != null) {
            chart.addSeries("forward", fwdSeg.t, fwdY).setMarker(new None());
            double minF = Arrays.stream(fwdY).min().orElse(0.0);
            double maxF = Arrays.stream(fwdY).max().orElse(0.0);
            if (!yInit) {
                globalYMin = minF;
                globalYMax = maxF;
                yInit = true;
            } else {
                globalYMin = Math.min(globalYMin, minF);
                globalYMax = Math.max(globalYMax, maxF);
            }
            tMax = Math.max(tMax, fwdSeg.t[fwdSeg.t.length - 1]);
        }

        if (!yInit) {
            System.err.println("Nothing to plot for " + yLabel);
            return;
        }

        double tAxisMax = Math.ceil(tMax / 0.5) * 0.5;
        double yAxisMin = Math.floor(globalYMin / 5.0) * 5.0;
        double yAxisMax = Math.ceil(globalYMax / 5.0) * 5.0;

        chart.getStyler().setXAxisMin(0.0);
        chart.getStyler().setXAxisMax(tAxisMax);
        chart.getStyler().setYAxisMin(yAxisMin);
        chart.getStyler().setYAxisMax(yAxisMax);
        chart.getStyler().setPlotGridLinesVisible(true);

        // brake lines
        if (backSeg != null && backSeg.brakeTime != null) {
            double bt = backSeg.brakeTime;
            double[] bx = { bt, bt };
            double[] by = { yAxisMin, yAxisMax };
            chart.addSeries("brake_back", bx, by).setMarker(new None());
        }
        if (fwdSeg != null && fwdSeg.brakeTime != null) {
            double ft = fwdSeg.brakeTime;
            double[] fx = { ft, ft };
            double[] fy = { yAxisMin, yAxisMax };
            chart.addSeries("brake_fwd", fx, fy).setMarker(new None());
        }

        // save & show
        try {
            BitmapEncoder.saveBitmapWithDPI(
                    chart,
                    fileName.replace(".png", ""),
                    BitmapFormat.PNG,
                    300
            );
            System.out.println("Saved: " + fileName);
        } catch (Exception e) {
            System.err.println("Error saving " + fileName + ": " + e.getMessage());
        }

        new SwingWrapper<>(chart).displayChart();
    }
}
