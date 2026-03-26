package frc.Lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class BlazeMath {
    public static double greaterMagnitude(double a, double b) {
        return Math.abs(b) > Math.abs(a) ? b : a;
    }

    public static double clampMagnitude(double input, double maxMagnitude) {
        return MathUtil.clamp(input, -maxMagnitude, maxMagnitude);
    }

    public static ChassisSpeeds changeToFieldRelative(ChassisSpeeds speeds, Rotation2d driveTrainAngle) {
        AdvancedPose2D vector = new AdvancedPose2D(speeds).addRotation(driveTrainAngle);
        return new ChassisSpeeds(vector.getX(), vector.getY(), vector.getRotation().getRadians());
    }
}
