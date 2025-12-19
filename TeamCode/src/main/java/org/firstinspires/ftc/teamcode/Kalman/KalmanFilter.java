package org.firstinspires.ftc.teamcode.Kalman;

public class KalmanFilter {
    // State vector: [x, y, heading]
    private double[] x = new double[3];
    private double[][] P = new double[3][3];
    private double[][] R = new double [3][3];
    private double[][] Q = new double [3][3];
    private double[][] F = new double[3][3];

    // Process noise & measurement noise
    private final double q;
    private final double r;

    public KalmanFilter(double processNoise, double measurementNoise) {
        this.q = processNoise;
        this.r = measurementNoise;

        // Initial covariance large (high uncertainty)
        P[0][0] = 1;
        P[1][1] = 1;
        P[2][2] = 1;

        // Measuremnt covariance matrix
        R[0][0] = Math.pow(0.003,2); // M
        R[1][1] = Math.pow(0.003,2);
        R[2][2] = Math.pow(0.005,2); //rad

        Q[0][0] = Math.pow(1,2); // M
        Q[1][1] = Math.pow(1,2);
        Q[2][2] = Math.pow(1,2); //rad
    }

    /** Prediction step using odometry deltas */
    public void predict(double dx, double dy, double dHeading) {
        // Predict state
        x[0] += dx*Math.cos(x[3])-dy*Math.sin(x[3]);
        x[1] += dx*Math.sin(x[3])+dy*Math.cos(x[3]);
        x[2] += -dHeading;



        // Increase uncertainty
        for (int i = 0; i < 3; i++) {
            P[i][i] += q;
        }
    }

    /** Update step with measurement (from IMU or external localizer) */
    public void update(double measX, double measY, double measHeading) {
        double[] z = { measX, measY, measHeading };

        // Kalman Gain K = P / (P + R)
        double[] K = new double[3];
        for (int i = 0; i < 3; i++) {
            K[i] = P[i][i] / (P[i][i] + r);
        }

        // Update estimate: x = x + K * (z − x)
        for (int i = 0; i < 3; i++) {
            x[i] += K[i] * (z[i] - x[i]);
            P[i][i] = (1 - K[i]) * P[i][i];
        }
    }

    public double getX() { return x[0]; }
    public double getY() { return x[1]; }
    public double getHeading() { return x[2]; }

    public void setState(double x, double y, double heading) {
        this.x[0] = x;
        this.x[1] = y;
        this.x[2] = heading;
    }
}
