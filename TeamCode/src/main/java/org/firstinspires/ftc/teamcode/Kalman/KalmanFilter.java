package org.firstinspires.ftc.teamcode.Kalman;

public class KalmanFilter {

    // State: [x, y, heading]
    private double[] x = new double[3];

    // Covariances
    private double[][] P = new double[3][3];
    private double[][] Q = new double[3][3];
    private double[][] R = new double[3][3];
    private double[][] F = new double[3][3];

    public KalmanFilter() {

        // Initial uncertainty
        P[0][0] = 1;
        P[1][1] = 1;
        P[2][2] = 1;

        // Measurement noise (tune if needed)
        R[0][0] = Math.pow(0.003, 2);
        R[1][1] = Math.pow(0.003, 2);
        R[2][2] = Math.pow(0.005, 2);
    }

    // ===================== PREDICT =====================
    public void predict(double dx, double dy, double dHeading) {

        double h = x[2];

        // --- State prediction ---
        x[0] += dx * Math.cos(h) - dy * Math.sin(h);
        x[1] += dx * Math.sin(h) + dy * Math.cos(h);
        x[2] += -dHeading;

        // --- Jacobian F ---
        F[0][0] = 1;  F[0][1] = 0;
        F[0][2] = -dx * Math.sin(h) - dy * Math.cos(h);

        F[1][0] = 0;  F[1][1] = 1;
        F[1][2] =  dx * Math.cos(h) - dy * Math.sin(h);

        F[2][0] = 0;  F[2][1] = 0;  F[2][2] = 1;

        // --- Dynamic Q = diag(sig^2) ---
        double dist = Math.hypot(dx, dy);

        double kPos = 0.02;
        double kTheta = 0.01;

        double sigX = kPos * dist;
        double sigY = kPos * dist;
        double sigTheta = kTheta * Math.abs(dHeading);

        Q[0][0] = sigX * sigX;
        Q[0][1] = 0;
        Q[0][2] = 0;

        Q[1][0] = 0;
        Q[1][1] = sigY * sigY;
        Q[1][2] = 0;

        Q[2][0] = 0;
        Q[2][1] = 0;
        Q[2][2] = sigTheta * sigTheta;

        // --- P = F P F^T + Q ---
        double[][] Ft = transpose(F);
        P = add(multiply(multiply(F, P), Ft), Q);
    }

    // ===================== UPDATE =====================
    public void update(double mx, double my, double mh) {

        double[] z = {mx, my, mh};

        // S = P + R  (H = I)
        double[][] S = add(P, R);
        double[][] S_inv = inverse3x3(S);

        // K = P S^-1
        double[][] K = multiply(P, S_inv);

        // y = z - x
        double[] y = new double[3];
        for (int i = 0; i < 3; i++)
            y[i] = z[i] - x[i];

        // x = x + K y
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                x[i] += K[i][j] * y[j];

        // P = (I - K) P
        double[][] I = {
                {1,0,0},
                {0,1,0},
                {0,0,1}
        };

        double[][] IK = subtract(I, K);
        P = multiply(IK, P);
    }

    // ===================== MATRIX HELPERS =====================

    private double[][] add(double[][] A, double[][] B) {
        double[][] C = new double[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                C[i][j] = A[i][j] + B[i][j];
        return C;
    }

    private double[][] subtract(double[][] A, double[][] B) {
        double[][] C = new double[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                C[i][j] = A[i][j] - B[i][j];
        return C;
    }

    private double[][] multiply(double[][] A, double[][] B) {
        double[][] C = new double[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                    C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    private double[][] transpose(double[][] A) {
        double[][] T = new double[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                T[i][j] = A[j][i];
        return T;
    }

    private double[][] inverse3x3(double[][] m) {
        double a=m[0][0], b=m[0][1], c=m[0][2];
        double d=m[1][0], e=m[1][1], f=m[1][2];
        double g=m[2][0], h=m[2][1], i=m[2][2];

        double A =   e*i - f*h;
        double B = -(d*i - f*g);
        double C =   d*h - e*g;
        double D = -(b*i - c*h);
        double E =   a*i - c*g;
        double F = -(a*h - b*g);
        double G =   b*f - c*e;
        double H = -(a*f - c*d);
        double I =   a*e - b*d;

        double det = a*A + b*B + c*C;

        double[][] inv = {
                {A/det, D/det, G/det},
                {B/det, E/det, H/det},
                {C/det, F/det, I/det}
        };
        return inv;
    }

    // ===================== GETTERS =====================
    public double getX() { return x[0]; }
    public double getY() { return x[1]; }
    public double getHeading() { return x[2]; }

    public void setState(double x, double y, double heading) {
        this.x[0] = x;
        this.x[1] = y;
        this.x[2] = heading;
    }
}
