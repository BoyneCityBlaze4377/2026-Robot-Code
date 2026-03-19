package frc.Lib;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class BlazeMath {
    public static double greaterMagnitude(double a, double b) {
        return Math.abs(b) > Math.abs(a) ? b : a;
    }

    public static double clampMagnitude(double input, double maxMagnitude) {
        return MathUtil.clamp(input, -maxMagnitude, maxMagnitude);
    }
}
