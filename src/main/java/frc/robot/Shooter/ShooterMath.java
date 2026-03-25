// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.Lib.AdvancedPose2D;
import frc.Lib.Vector3D;
import frc.Lib.Zone;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.DriveTrain.DriveTrain;
import frc.robot.DriveTrain.DriveTrain.DriveTrainZoneState;

/** Add your docs here. */
public class ShooterMath {
    public static ShooterState calcDesiredState(AdvancedPose2D driveTrainPos, 
                                                ChassisSpeeds driveTrainSpeeds,
                                                Vector3D driveTrainOmega,
                                                Pose3d targetPose, 
                                                double shotVelocity) {

        Vector3D driveTrainVelocityVector = new Vector3D(driveTrainSpeeds);

        AdvancedPose2D currentPosition = driveTrainPos.withVector(driveTrainPos.getRotation(), 
                                                AutoAimConstants.turretOffsetCoordinates.getTranslation(), 
                                                new Rotation2d());
        Pose3d currentPosition3D = new Pose3d(currentPosition.getX(), currentPosition.getY(), 
                                              AutoAimConstants.turretOffsetPos.getZ(), new Rotation3d());
        
        Vector3D currentVelocity2D = Vector3D.getPointVelocity(driveTrainVelocityVector, 
                                                               new Vector3D(currentPosition), 
                                                               driveTrainOmega);   
                                                    
        Vector3D currentVelocity = driveTrainVelocityVector; //new Vector3D(new AdvancedPose2D().withVector(driveTrainPos.getRotation(), 
            //new Translation2d(currentVelocity2D.getX(), currentVelocity2D.getY()), new Rotation2d()));
                                                
        Vector3D turretAimVector = Vector3D.fromPoints(currentPosition3D, targetPose).minus(currentVelocity);
        Rotation2d turretAngle = turretAimVector.getXYAngle().minus(driveTrainPos.getRotation());

        double horizDistance = new AdvancedPose2D(targetPose).getDistance(currentPosition);
        double vertDistance = targetPose.getZ() - currentPosition3D.getZ();


        double a = -4.9 * Math.pow(horizDistance / shotVelocity, 2);
        double b = horizDistance;
        double c = -(4.9 * Math.pow(horizDistance / shotVelocity, 2) + vertDistance);

        double quadFormSolution = (-b - Math.sqrt(Math.pow(b, 2) - 4 * a * c)) / (2 * a);

        Rotation2d hoodAngle = Rotation2d.fromRadians(Math.atan(quadFormSolution));

        if (hoodAngle.getDegrees() <= 90 - ShooterConstants.minHoodHeight || Math.abs(hoodAngle.getDegrees()) > 1e9) {
            hoodAngle = Rotation2d.fromDegrees(90 - ShooterConstants.minHoodHeight + .5);
            shotVelocity = findVFromThetaAndD(hoodAngle, horizDistance);
        }

        return new ShooterState(turretAngle, hoodAngle, shotVelocity, true);
    }

    public static double findVFromThetaAndD(Rotation2d theta, double distance) {
        return Math.sqrt(9.8 * distance / Math.sin(2 * theta.getRadians()));
    }

    public static boolean moveIsTooBig(double currentPos, double desiredPos) {
        return Math.abs(currentPos - desiredPos) > ShooterConstants.moveTypeThreshold;
    }

    public static boolean tooCloseToTrench(AdvancedPose2D position, AdvancedPose2D trenchPos) {
        return position.getDistance(trenchPos) <= AutoAimConstants.trenchDistanceThreshold;
    }

    public static boolean movingTowardsTrench(AdvancedPose2D position, ChassisSpeeds velocity, AdvancedPose2D trenchPos) {
        return Math.abs(new Vector3D(velocity).getXYAngle().getDegrees() 
                            - Vector3D.fromPoints(position, trenchPos).getXYAngle().getDegrees()) 
               <= AutoAimConstants.trenchAngleThreshold;
    }
}
