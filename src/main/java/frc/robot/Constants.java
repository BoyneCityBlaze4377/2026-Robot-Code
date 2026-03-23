package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.Lib.AdvancedPose2D;
import frc.Lib.Vector3D;
import frc.Lib.Zone;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public class IOConstants {
    /* DASHBOARD TABS */
    public static final ShuffleboardTab TeleopTab = Shuffleboard.getTab("Teleop");
    public static final ShuffleboardTab AutonTab = Shuffleboard.getTab("Auton");
    public static final ShuffleboardTab DiagnosticTab = Shuffleboard.getTab("Diagnostic");
    public static final ShuffleboardTab ConfigTab = Shuffleboard.getTab("Configuration");

    /* CONTROLLER IDS */
    public static final int driverControllerID = 0;

    /* BUTTON IDS */
    /* Driver */
    //Main Functions
    public static final int quickBrakeButtonID = 1; //6
    public static final int slowModeButtonID = 3; //3
    public static final int switchOrientationButtonID = 4; //4
    public static final int lockPoseButtonID = 5; //5
    public static final int robotOrientButtonID = 9; //9

    //Driving Axes
    public static final int xAxis = 1;
    public static final int yAxis = 0;
    public static final int omegaAxis = 4;
  }

  /* Physical constants of the DriveTrain */
  public static final class SwerveConstants {
    // Distance between centers of right and left wheels on robot in meters
    public static final double trackWidth = Units.inchesToMeters(20.5); //.9144
    
    // Distance between front and back wheels on robot in meters
    public static final double wheelBase = Units.inchesToMeters(20.5); //.9144

    public static final double bumperthickness = Units.inchesToMeters(0);
    public static final double robotWidth = Units.inchesToMeters(27.5) + bumperthickness;
    
    public static final SwerveDriveKinematics driveKinematics =
        new SwerveDriveKinematics(new Translation2d(wheelBase / 2, trackWidth / 2),
                                  new Translation2d(wheelBase / 2, -trackWidth / 2),
                                  new Translation2d(-wheelBase / 2, trackWidth / 2),
                                  new Translation2d(-wheelBase / 2, -trackWidth / 2));
  }

  /* Constants related to how the DriveTrain moves */
  public static final class DriveConstants {
    public static final double speedScaler = .25;

    public static final double maxSpeedMetersPerSecond = 5;
    public static final double maxAccelerationMetersPerSecondSquared = 4;
    public static final double maxRotationSpeedRadiansPerSecond = 3 * Math.PI;
    public static final double maxRotationAccelerationRadiansPerSecondSquared = 2 * maxRotationSpeedRadiansPerSecond;

    public static final double translationalDeadband = .15;
    public static final double rotationalDeadband = .3;

    public static final double jerkCrashTheshold = maxAccelerationMetersPerSecondSquared * Math.sqrt(2);
    public static final double crashDebounceTime = .1;

    public static final boolean gyroReversed = false;
  }

  /* Constants to create and configure SwerveModules */
  public static final class ModuleConstants {
    /** CAN ID's */
    public static final int frontLeftDriveMotorID = 1;
    public static final int frontRightDriveMotorID = 3;
    public static final int backLeftDriveMotorID = 7;
    public static final int backRightDriveMotorID = 5;

    public static final int frontLeftTurningMotorID = 2;
    public static final int frontRightTurningMotorID = 4;
    public static final int backLeftTurningMotorID = 8;
    public static final int backRightTurningMotorID = 6;
    
    public static final int frontLeftTurningEncoderID = 9;
    public static final int frontRightTurningEncoderID = 10;
    public static final int backLeftTurningEncoderID = 12;
    public static final int backRightTurningEncoderID = 11;

    /** Inversions */
    public static final boolean frontLeftTurningMotorReversed = false;    
    public static final boolean frontRightTurningMotorReversed = false;
    public static final boolean backLeftTurningMotorReversed = false;
    public static final boolean backRightTurningMotorReversed = false;

    public static final boolean frontLeftDriveMotorReversed = false;    
    public static final boolean frontRightDriveMotorReversed = false;
    public static final boolean backLeftDriveMotorReversed = false;
    public static final boolean backRightDriveMotorReversed = false;
    
    public static final boolean frontLeftAbsReversed = false;    
    public static final boolean frontRightAbsReversed = false;
    public static final boolean backLeftAbsReversed = false;
    public static final boolean backRightAbsReversed = false;

    /** Offsets */
    public static final double frontLeftAbsoluteEncoderOffset = -166.35;  
    public static final double frontRightAbsoluteEncoderOffset = 47.28;
    public static final double backLeftAbsoluteEncoderOffset = -134.73;
    public static final double backRightAbsoluteEncoderOffset = 12.74;

    /** Full Module */
    public static final double maxModuleSpeedMetersPerSecond = 5; //5.614416 True max
    public static final double maxModuleAccelerationMetersPerSecondSquared = 7;
    public static final double maxModuleAngularSpeedDegreesPerSecond = 360 * 3;
    public static final double maxModuleAngularAccelerationDegreesPerSecondSquared = maxModuleAngularSpeedDegreesPerSecond * 3;

    public static final double kMaxOutput = 0.1;
    public static final double maxVoltage = 14.0;
    public static final int angleContinuousCurrentLimit = 20;

    public static final double absoluteEncoderRange = .5; // [-.5, .5)

    /** Drive Motor */
    public static final double wheelDiameterMeters = Units.inchesToMeters(4);
    public static final double driveGearRatio = 5.68;
    public static final double driveMotorConversionFactor = (wheelDiameterMeters * Math.PI) / driveGearRatio;

    public static final double driveKP = 0.4; //.01
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKTolerance = .5;

    public static final NeutralModeValue initialDriveNeutralMode  = NeutralModeValue.Brake;

    /** Turning motor */
    public static final double angleGearRatio = 12.1;
    public static final double angleConversionFactor = angleGearRatio / 360;
  
    public static final Constraints angleControllerConstraints = new Constraints(maxModuleAngularSpeedDegreesPerSecond, 
                                                                                 maxModuleAngularAccelerationDegreesPerSecondSquared);
    public static final double angleKP = 0.005; //.005
    public static final double angleKI = 0.00002; //.00002
    public static final double angleKD = 0.00009; //.00009
    public static final double angleKTolerance = .5;
  
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
  }

  public static final class ShooterConstants {
    /* ID's */
    public static final int flyWheelMotor1ID = 14;
    public static final int flyWheelMotor2ID = 15;
    public static final int spindexerID = 19;
    public static final int indexerID = 20;
    public static final int turretMotorID = 13;  
    public static final int hoodMotorID = 16;

    /* NeutralModes */
    public static final NeutralModeValue turretNeutralModeValue = NeutralModeValue.Brake;
    public static final NeutralModeValue hoodNeutralModeValue = NeutralModeValue.Brake;
    
    /* Speed and Control */
    
    public static final double minVelocity = .3;
    public static final double maxVelocity = .8;
    public static final double velocityAddOn = 0; 
    // public static final double revFlywheelSpeed = .85;
    public static final double spindexerSpeed = .75;
    public static final double indexingSpeed = .5;

    public static final double shooterMaxVoltage = 16;
    public static final double shooterMaxDutyCycle = 1;

    public static final double bigMoveAimingKP = 0.006; //.025
    public static final double bigMoveAimingKI = 0.000008; //5e-7
    public static final double bigMoveAimingKD = 0.000015; //1

    public static final double fineTuneAimingKP = 0.025; //.1
    public static final double finetuneAimingKI = 0.000001; //5e-7
    public static final double fineTuneAimingKD = 0.000000001; //1

    public static final double aimingKTolerance = .5;
    public static final double moveTypeThreshold = 10;

    public static final double maxTurretOutput = .75;
    //public static final Constraints aimingControllerConstraints = new Constraints(0, 0);

    public static final double turretMaxVoltage = 16;
    public static final double turretMaxDutyCycle = 1;

    public static final double hoodKP = 0.008; //.01
    public static final double hoodKI = 0.0;
    public static final double hoodKD = 0.000;
    public static final double hoodKTolerance = .5;
    public static final Constraints hoodControllerConstraints = new Constraints(0, 0);

    public static final double hoodMaxVoltage = 16;
    public static final double hoodMaxDutyCycle = 1;

    public static final double maxHoodHeight = 0;
    public static final double minHoodHeight = 0;

    public static final double turretAngleOffset = 125;

    /* Aiming */
    public static final double turretGearRatio = 96 / 25;
    public static final double aimingConversionFactor = 93.75; //360 / turretGearRatio;

    public static final double hoodGearRatio = 1;
    public static final double hoodConversionFactor = 37.5 / 1.303;
  }

  public static final class CollectorConstants {
    /* ID's */
    public static final int collectorMotorID = 18;
    public static final int deployMotorID = 17;

    /* NeutralModes */
    public static final IdleMode collectorMotorNeutralMode = IdleMode.kCoast;
    public static final NeutralModeValue deployMotorNeutralMode = NeutralModeValue.Brake;

    /* Speed and Control */
    public static final double collectionSpeed = -.25;
    public static final double maxDeploySpeed = .5;

    public static final double deployGearRatio = 1;
    public static final double deployConversionFactor = Units.inchesToMeters(13) / 15.311;

    public static final double deployMotorMaxDutyCycle = 1;
    public static final double deployMotorMaxVoltage = 12;

    public static final double deployKP = 1.2;
    public static final double deployKI = 0;
    public static final double deployKD = 0;

    public static final double deployedPos = .37559;
    public static final double retractedPos = 0;
    public static final double setpointOffset = .01;

    public static final double jostleInterval = .5;
    public static final double jostleSpeedDifferential = .05;
    public static final double closeToExtendedTolerance = 0;
  }

  public static final class ClimberConstants {
    public static final int climberMotorID = 21;

    public static final double climberMaxVoltage = 16;
    public static final double climberMaxDutyCycle = 1;

    public static final double maxHeightPos = 0;
    public static final double minHeightPos = 0;

    public static final double climberKP = 0.012;
    public static final double climberKI = 0;
    public static final double climberKD = 0;
  }

  public static final class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(651.22);
    public static final double fieldWidth = Units.inchesToMeters(317.69);

    public static final double autonLineDistance = Units.inchesToMeters(156.06);
    public static final double hubOpeningHeight = Units.inchesToMeters(72);
    public static final double netHeight = Units.inchesToMeters(120.36);
    public static final double hubCenterX = Units.inchesToMeters(182.11);

    public static final double trenchWidth = Units.inchesToMeters(50.35);

    public static final AdvancedPose2D hubCoordinates = new AdvancedPose2D(Units.inchesToMeters(182.11), fieldWidth / 2);
    public static final Pose3d hubPosition = new Pose3d(hubCoordinates.getX(), 
                                                        hubCoordinates.getY(), 
                                                        hubOpeningHeight, 
                                                        new Rotation3d());

    public static final AdvancedPose2D leftShuttleTarget = new AdvancedPose2D(autonLineDistance / 2, fieldWidth * 3 / 4);
    public static final AdvancedPose2D rightShuttleTarget = new AdvancedPose2D(autonLineDistance / 2, fieldWidth / 4);

    public static final AdvancedPose2D outpostPos = new AdvancedPose2D(SwerveConstants.robotWidth, SwerveConstants.robotWidth);

    public static final Zone allianceZone = new Zone(new AdvancedPose2D(), 
                                                     new AdvancedPose2D(hubCenterX, fieldWidth));
    public static final Zone neutralZone = new Zone(new AdvancedPose2D(hubCenterX, 0), 
                                                    new AdvancedPose2D(fieldLength - hubCenterX, fieldWidth));
    public static final Zone trenchZone = new Zone(new AdvancedPose2D(autonLineDistance, 0), 
                                                   new AdvancedPose2D(fieldLength/2 - 120, fieldWidth));
    public static final Zone rightTrench = new Zone(trenchZone.getBL(), new AdvancedPose2D(fieldLength/2 - 120, trenchWidth));
    public static final Zone leftTrench = new Zone(new AdvancedPose2D(autonLineDistance, fieldWidth - trenchWidth), 
                                                   trenchZone.getTR());

  }

  public class AutoAimConstants {
    public static final double transkP = 2; //1.5
    public static final double transkI = 0; //.013
    public static final double transkD = 0; //0
    public static final double transkTolerance = .025;

    public static final double turnkP = 4.5; //4.5
    public static final double turnkI = .355; //.355
    public static final double turnkD = 0; //0
    public static final double turnkTolerance = .03;

    //PathPlanner use
    public static final double PPtranskP = 2;//2
    public static final double PPtranskI = 0;
    public static final double PPtranskD = 0;

    public static final double PPturnkP = 4.5;//4.5
    public static final double PPturnkI = .355;//.355
    public static final double PPturnkD = 0;

    public static final AdvancedPose2D turretOffsetCoordinates = new AdvancedPose2D(.5, .5);
    public static final Vector3D turretOffsetPos = new Vector3D(.5, .5, .2);

    public static final double dotProductThreshold = .25;

    public static final Vector<N3> poseEstimateOdometryStdDev = VecBuilder.fill(.05, .05, Units.degreesToRadians(.2));
    public static final Vector<N3> poseEstimateVisionStdDev = VecBuilder.fill(.1, .1, Units.degreesToRadians(3));
    public static final Vector<N3> poseEstimateCrashVisionStdDev = VecBuilder.fill(.02, .02, Units.degreesToRadians(2));
  }

  public class AutonConstants {}

  public class SensorConstants {
    /** LIMELIGHT */
    public static final String limeLightName = "limelight";
  }

  public class PathPlaner {
    public static RobotConfig config;
  }
}