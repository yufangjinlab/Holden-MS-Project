package org.firstinspires.ftc.teamcode.Kalman;

public class KalmanFilterPose {

    public double x, y, h;

    private double[][] P = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1}
    };

    private final double Q = 0.01;
    private final double R = 0.15;

    public KalmanFilterPose(double x0, double y0, double h0) {
        x = x0;
        y = y0;
        h = h0;
    }

    public void predict(double dx, double dy, double dh) {
        x += dx;
        y += dy;
        h = wrap(h + dh);

        for (int i = 0; i < 3; i++)
            P[i][i] += Q;
    }

    public void update(double mx, double my, double mh) {
        double[] z = {mx, my, mh};
        double[] s = {x, y, h};

        double[] innovation = {
                z[0] - s[0],
                z[1] - s[1],
                wrap(z[2] - s[2])
        };

        double[][] K = new double[3][3];
        for (int i = 0; i < 3; i++) {
            double denom = P[i][i] + R;
            K[i][i] = P[i][i] / denom;
        }

        x += K[0][0] * innovation[0];
        y += K[1][1] * innovation[1];
        h = wrap(h + K[2][2] * innovation[2]);

        for (int i = 0; i < 3; i++)
            P[i][i] *= (1 - K[i][i]);
    }

    private double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    // === ADD THESE ===
    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return h; }
}
