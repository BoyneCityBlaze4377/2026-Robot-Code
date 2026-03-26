package frc.robot.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class ShooterState {
    public Rotation2d angleToTarget = new Rotation2d(), shotPitch = new Rotation2d();
    public double shotVelocity;
    public boolean isShooting;

    public ShooterState(Rotation2d angle, Rotation2d pitch, double velocity, boolean shooting) {
        angleToTarget = angle;
        shotPitch = pitch;
        shotVelocity = velocity;
        isShooting = shooting;
    }

    public ShooterState(Rotation2d angle, double velocity) {
        this(angle, new Rotation2d(), velocity, false);
    }

    public ShooterState(Rotation2d angle) {
        this(angle, 0);
    }

    private void setShooting(boolean shooting) {
        isShooting = shooting;
    }

    public ShooterState noShooting() {
        this.setShooting(false);
        return this;
    }

    public String toString() {
        return "ShooterState(" + angleToTarget.getDegrees() + " degrees to target, " 
                               + shotPitch.getDegrees() + " degrees above ground, " 
                               + shotVelocity + " m/s, "
                               + (isShooting ? "shooting" : "not shooting") 
                               + ")";
    }
}
