// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Climber;

import frc.robot.Constants.ClimberConstants;

/** Add your docs here. */
public class ClimberState {
    public double position;

    public static final ClimberState retracted = new ClimberState(ClimberConstants.minHeightPos);
    public static final ClimberState extended = new ClimberState(ClimberConstants.maxHeightPos);
    public static final ClimberState climbed = new ClimberState(ClimberConstants.climbedHeightPos);

    public ClimberState(double height) {
        position = height;
    }
}
