package org.firstinspires.ftc.teamcode.BBQ;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterMath {
    public static double HOOD_MIN = 1.0;
    public static double HOOD_MAX = 0.22;
    public static double RPM_MIN = 1000.0;
    public static double RPM_MAX = 6000.0;

    private static final double[] RPM_DISTANCES = {15,   30,   45,   60,   75,   90,   105,  120};
    private static final double[] RPM_VALUES    = {1000, 1090, 1150, 1250, 1350, 1500, 1580, 2000};

    private static final double[] HOOD_DISTANCES = {15,  30,  45,   60,  75,  90,   105,  120};
    private static final double[] HOOD_VALUES    = {1.0, 0.6, 0.55, 0.4, 0.3, 0.25, 0.22, 0.22};

    public static double flywheelRPM(double dist) {
        double rpm = interpolateLUT(dist, RPM_DISTANCES, RPM_VALUES);
        return clamp(rpm, RPM_MIN, RPM_MAX);
    }

    public static double hoodPos(double dist) {
        double pos = interpolateLUT(dist, HOOD_DISTANCES, HOOD_VALUES);
        return clampDirectional(pos, HOOD_MIN, HOOD_MAX);
    }

    private static double interpolateLUT(double x, double[] lutX, double[] lutY) {
        if (x <= lutX[0]) return lutY[0];
        if (x >= lutX[lutX.length - 1]) return lutY[lutY.length - 1];

        for (int i = 0; i < lutX.length - 1; i++) {
            if (x >= lutX[i] && x <= lutX[i + 1]) {
                double portion = (x - lutX[i]) / (lutX[i + 1] - lutX[i]);
                return lutY[i] + portion * (lutY[i + 1] - lutY[i]);
            }
        }
        return lutY[0];
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(v, max));
    }

    private static double clampDirectional(double v, double limitA, double limitB) {
        double realMin = Math.min(limitA, limitB);
        double realMax = Math.max(limitA, limitB);
        return Math.max(realMin, Math.min(v, realMax));
    }
}