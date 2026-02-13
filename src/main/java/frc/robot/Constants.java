package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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
    public static final int omegaAxis = 2;
  }

  /* Physical constants of the DriveTrain */
  public static final class SwerveConstants {
    // Distance between centers of right and left wheels on robot in meters
    public static final double trackWidth = Units.inchesToMeters(36); //.9144
    
    // Distance between front and back wheels on robot in meters
    public static final double wheelBase = Units.inchesToMeters(36); //.9144
    
    public static final SwerveDriveKinematics driveKinematics =
        new SwerveDriveKinematics(new Translation2d(wheelBase / 2, trackWidth / 2),
                                  new Translation2d(wheelBase / 2, -trackWidth / 2),
                                  new Translation2d(-wheelBase / 2, trackWidth / 2),
                                  new Translation2d(-wheelBase / 2, -trackWidth / 2));
  }

  /* Constants related to how the DriveTrain moves */
  public static final class DriveConstants {
    public static final double speedScaler = 1;

    public static final double maxSpeedMetersPerSecond = 5;
    public static final double maxAccelerationMetersPerSecondSquared = 4;
    public static final double maxRotationSpeedRadiansPerSecond = 3 * Math.PI;
    public static final double maxRotationAccelerationRadiansPerSecondSquared = 2 * maxRotationSpeedRadiansPerSecond;

    public static final double translationalDeadband = .15;
    public static final double rotationalDeadband = .4;

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
    public static final double frontLeftAbsoluteEncoderOffset = -168.22;  
    public static final double frontRightAbsoluteEncoderOffset = 44.29;
    public static final double backLeftAbsoluteEncoderOffset = -136.66;
    public static final double backRightAbsoluteEncoderOffset = 13.09;

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
    public static final int shooterMotorID = 0;
    public static final int indexerMotorID = 0;
    public static final int turretMotorID = 0;  
    public static final int hoodMotorID = 0;

    public static final int aimingEncoderID = 0;
    public static final int hoodEncoderID = 0;
    public static final int flywheelEncoderID = 0;

    /* NeutralModes */
    public static final NeutralModeValue shootingMotorNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue indexingNeutralModeValue = NeutralModeValue.Brake;
    public static final NeutralModeValue turretNeutralModeValue = NeutralModeValue.Brake;
    public static final NeutralModeValue hoodNeutralModeValue = NeutralModeValue.Brake;
    
    /* Speed and Control */
    public static final double indexingSpeed = .5;
    public static final double timeBetweenShots = .1;

    public static final double shootingKP = 0.1; //.01
    public static final double shootingKI = 0.0;
    public static final double shootingKD = 0.0;
    public static final double shootingKTolerance = .5; 

    public static final double shooterMaxVoltage = 16;
    public static final double shooterMaxDutyCycle = 1;

    public static final double aimingKP = 0.1; //.01
    public static final double aimingKI = 0.0;
    public static final double aimingKD = 0.0;
    public static final double aimingKTolerance = .5;
    public static final Constraints aimingControllerConstraints = new Constraints(Math.PI * 4, Math.PI * 8);

    public static final double turretMaxVoltage = 16;
    public static final double turretMaxDutyCycle = 1;

    public static final double hoodKP = 0.1; //.01
    public static final double hoodKI = 0.0;
    public static final double hoodKD = 0.0;
    public static final double hoodKTolerance = .5;
    public static final Constraints hoodControllerConstraints = new Constraints(Math.PI * 4, Math.PI * 8);

    public static final double hoodMaxVoltage = 16;
    public static final double hoodMaxDutyCycle = 1;

    public static final double maxHoodHeight = 0;
    public static final double minHoodHeight = 0;

    /* Aiming */
    public static final SensorDirectionValue aimingEncoderSensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    public static final double aimingEncoderRange = .5; //[-.5, .5)

    public static final double turretGearRatioFromEncoder = 1;
    public static final double aimingConversionFactor = 360 / turretGearRatioFromEncoder;

    public static final double hoodGearRatioFromEncoder = 1;
    public static final double hoodConversionFactor = 360 / hoodGearRatioFromEncoder;
  }

  public static final class CollectorConstants {
    /* ID's */
    public static final int collectorMotorID = 0;
    public static final int deployMotorID = 0;

    /* NeutralModes */
    public static final IdleMode collectorMotorNeutralMode = IdleMode.kCoast;
    public static final NeutralModeValue deployMotorNeutralMode = NeutralModeValue.Brake;

    /* Speed and Control */
    public static final double collectionSpeed = .5;

    public static final double deployMotorMaxDutyCycle = 1;
    public static final double deployMotorMaxVoltage = 12;

    public static final double deployKP = 0;
    public static final double deployKI = 0;
    public static final double deployKD = 0;

    public static final double deployedPos = 0;
    public static final double retractedPos = 0;
  }

  public static final class ClimberConstants {
    public static final int climberMotorID = 0;
    public static final int climberEncoderID = 0;

    public static final double climberMaxVoltage = 16;
    public static final double climberMaxDutyCycle = 1;

    public static final SensorDirectionValue climberEncoderSensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    public static final double maxExtensionPos = 0;
    public static final double mediumClimbPos = 0;
    public static final double minHeightPos = 0;

    public static final double climberKP = 0;
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

    public static final AdvancedPose2D hubCoordinates = new AdvancedPose2D(Units.inchesToMeters(182.11), fieldWidth / 2);
    public static final Pose3d hubPosition = new Pose3d(hubCoordinates.getX(), 
                                                        hubCoordinates.getY(), 
                                                        hubOpeningHeight, 
                                                        new Rotation3d());
    public static final AdvancedPose2D leftShuttleTarget = new AdvancedPose2D(autonLineDistance / 2, fieldWidth * 3 / 4);
    public static final AdvancedPose2D rightShuttleTarget = new AdvancedPose2D(autonLineDistance / 2, fieldWidth / 4);

    public static final Zone allianceZone = new Zone(new AdvancedPose2D(), 
                                                     new AdvancedPose2D(hubCenterX, fieldWidth));
    public static final Zone neutralZone = new Zone(new AdvancedPose2D(hubCenterX, 0), 
                                                    new AdvancedPose2D(fieldLength - hubCenterX, fieldWidth));
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

    public static final AdvancedPose2D turretOffsetCoordinates = new AdvancedPose2D();
    public static final Vector3D turretOffsetPos = new Vector3D();

    public static final double maxShotHeight = FieldConstants.hubOpeningHeight + 1;

    public static final Vector<N3> poseEstimateOdometryStdDev = VecBuilder.fill(.05, .05, Units.degreesToRadians(.2));
    public static final Vector<N3> poseEstimateVisionStdDev = VecBuilder.fill(.1, .1, Units.degreesToRadians(3));
    public static final Vector<N3> poseEstimateCrashVisionStdDev = VecBuilder.fill(.02, .02, Units.degreesToRadians(2));
  }

  public class AutonConstants {}

  public class SensorConstants {
    /** LIMELIGHT */
    public static final String limeLightName = "limelight";
  }
}