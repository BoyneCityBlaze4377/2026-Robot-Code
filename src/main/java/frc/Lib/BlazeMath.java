package frc.Lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class BlazeMath {
    public static double greaterMagnitude(double a, double b) {
        return Math.abs(b) > Math.abs(a) ? b : a;
    }

    public static double clampMagnitude(double input, double maxMagnitude) {
        return MathUtil.clamp(input, -maxMagnitude, maxMagnitude);
    }

    public static double roundToDecimalPlaces(double value, double numDecimalPlaces) {
        return Math.round(value * pow(10, numDecimalPlaces)) / pow(10, numDecimalPlaces);
    }

    private static double sin(double a) {
        return Math.sin(a);
    }

    private static double cos(double a) {
        return Math.cos(a);
    }

    private static double tan(double a) {
        return Math.tan(a);
    }

    private static double sqrt(double a) {
        return Math.sqrt(a);
    }

    private static double pow(double a, double b) {
        return Math.pow(a, b);
    }

    public static double shotDistance(double theta, double v) {
        return v * cos(theta) * ((v * sin(theta) + sqrt(pow(v * sin(theta), 2) - 29.4)) / 9.8);
    }

    public static double dDdTheta(double theta, double v) {
        double c = 0.1020408163;

        return c * v * cos(theta) * ((pow(v * sin(theta), 2) + 
                                          pow(v, 2) * sin(theta) * cos(theta)) / sqrt(pow(v * sin(theta), 2) - 29.4)
                                      + v * cos(theta) 
                                      + v * sin(theta))
               - c * v * sin(theta) * (sqrt(pow(v * sin(theta), 2) - 29.4) + v * sin(theta))
               + c * v * cos(theta) * (sqrt(pow(v * sin(theta), 2) - 29.4) + v * sin(theta));
    }

    public static double dDdV(double theta, double v) {
        double c = 0.1020408163;

        return c * v * cos(theta) * ((v * pow(cos(theta), 2) + theta * sin(theta) * cos(theta) * pow(sin(theta), 2))
                                          / sqrt(pow(v * sin(theta), 2) - 29.4)
                                      + theta * v * cos(theta) 
                                      + sin(theta))
               + c * cos(theta) * (sqrt(pow(v * sin(theta), 2) - 29.4) + v * sin(theta))
               - c * v * theta * sin(theta) * (sqrt(pow(v * sin(theta), 2) - 29.4) + v * sin(theta));
    }
}
