package frc.Lib;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Vector3D {
    private double x, y, z;

    public Vector3D(double xComponent, double yComponent, double zComponent) {
        x = xComponent;
        y = yComponent;
        z = zComponent;
    }

    public Vector3D(double xComponent, double yComponent) {
        this(xComponent, yComponent, 0);
    }

    public Vector3D(Pose3d components) {
        this(components.getX(), components.getY(), components.getZ());
    }

    public Vector3D(AdvancedPose2D coordinates) {
        this(coordinates.getX(), coordinates.getY());
    }

    public Vector3D() {
        this(0, 0, 0);
    }

    public static Vector3D fromPoints(Pose3d tail, Pose3d tip) {
        return new Vector3D(tip.getX() - tail.getX(), tip.getY() - tail.getY(), tip.getZ() - tail.getZ());
    }

    public static final Vector3D fromPoints(AdvancedPose2D tail, AdvancedPose2D tip) {
        return Vector3D.fromPoints(new Pose3d(tail), new Pose3d(tip));
    }

    public static final Vector3D getPointVelocity(Vector3D translationVelocity, 
                                                  Vector3D offsetFromAxis,
                                                  Vector3D angularVelocity) {
        //r x omega                                            
        Vector3D velocityFromRot = new Vector3D(offsetFromAxis.getY() * angularVelocity.getZ() - 
                                                    offsetFromAxis.getZ() * angularVelocity.getY(),
                                               -offsetFromAxis.getX() * angularVelocity.getZ() - 
                                                    offsetFromAxis.getZ() * angularVelocity.getX(),
                                                offsetFromAxis.getX() * angularVelocity.getY() - 
                                                    offsetFromAxis.getY() * angularVelocity.getX());
    
        return velocityFromRot.plus(translationVelocity);
    }

    public Vector3D minus(Vector3D other) {
        return new Vector3D(this.getX() - other.getX(), this.getY() - other.getY(), this.getZ() - other.getZ());
    }

    public Vector3D plus(Vector3D other) {
        return new Vector3D(this.getX() + other.getX(), this.getY() + other.getY(), this.getZ() + other.getZ());
    }

    public double getMagnitude() {
        return Math.sqrt(Math.pow(getX(), 2) + Math.pow(getY(), 2) + Math.pow(z, 2));
    }

    public double get2DMagnitude() {
        return Math.hypot(getX(), getY());
    }

    public Rotation2d getXYAngle() {
        return Rotation2d.fromRadians(Math.atan(getX() / getY()));
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }
}
