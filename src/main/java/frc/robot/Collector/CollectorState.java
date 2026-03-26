package frc.robot.Collector;

import frc.robot.Constants.CollectorConstants;

/** Add your docs here. */
public class CollectorState {
    public double position, velocity;

    public static final CollectorState collecting = new CollectorState(CollectorConstants.deployedPos, 
                                                                       CollectorConstants.collectionSpeed);

    public static final CollectorState fullyRetracted = new CollectorState(CollectorConstants.retractedPos);

    public CollectorState(double collectorPosition, double wheelVelocity) {
        position = collectorPosition;
        velocity = wheelVelocity;
    }

    public CollectorState(double collectorPosition) {
        this(collectorPosition, 0);
    }
}
