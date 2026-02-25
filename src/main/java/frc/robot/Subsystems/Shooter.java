package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Lib.AdvancedPose2D;
import frc.Lib.Vector3D;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.DriveTrain.DriveTrainZoneState;

public class Shooter extends SubsystemBase {
  private final TalonFX m_turret, m_hood;
  private final TalonFXConfiguration m_turretConfig, m_hoodConfig;

  private final SparkFlex m_flyWheelMotor1, m_flyWheelMotor2;
  private final SparkFlexConfig m_flyWheel1Config, m_flyWheel2Config;

  private final SparkMax m_spindexer, m_indexer;
  private final SparkMaxConfig m_spindexerConfig, m_indexerConfig;

  private final CANcoder m_aimingEncoder, m_hoodEncoder;
  private final CANcoderConfiguration m_aimingEncoderConfig, m_hoodEncoderConfig;

  private final RelativeEncoder m_flywheelEncoder;

  private final ProfiledPIDController m_aimingController, m_hoodController;

  private final Supplier<AdvancedPose2D> m_driveTrainPositionSupplier;
  private final Supplier<ChassisSpeeds> m_driveTrainVelocitySupplier;
  private final Supplier<Vector3D> m_driveTrainAngularVelocitySupplier;
  private final Supplier<DriveTrain.DriveTrainZoneState> m_driveTrainZoneStateSupplier;

  private AdvancedPose2D driveTrainPos = new AdvancedPose2D();
  private ChassisSpeeds driveTrainSpeeds = new ChassisSpeeds();
  private Vector3D driveTrainOmega = new Vector3D();
  private DriveTrain.DriveTrainZoneState currentZone = DriveTrainZoneState.AllianceZone;

  private AdvancedPose2D currentPosition = new AdvancedPose2D();
  private Pose3d currentPosition3D = new Pose3d();
  private Vector3D currentVelocity = new Vector3D();

  private boolean canShoot = false;

  /** Creates a new Shooter. */
  public Shooter(Supplier<AdvancedPose2D> driveTrainPosition, 
                 Supplier<ChassisSpeeds> driveTrainVelocity,
                 Supplier<Vector3D> driveTrainAngularVelocity,
                 Supplier<DriveTrain.DriveTrainZoneState> zoneState) {
    m_turret = new TalonFX(ShooterConstants.turretMotorID);
    m_hood = new TalonFX(ShooterConstants.hoodMotorID);

    m_flyWheelMotor1 = new SparkFlex(ShooterConstants.flyWheelMotor1ID, MotorType.kBrushless);
    m_flyWheelMotor2 = new SparkFlex(ShooterConstants.flyWheelMotor2ID, MotorType.kBrushless);

    m_spindexer = new SparkMax(ShooterConstants.spindexerID, MotorType.kBrushless);
    m_indexer = new SparkMax(ShooterConstants.indexerID, MotorType.kBrushless);

    m_aimingEncoder = new CANcoder(ShooterConstants.aimingEncoderID);
    m_hoodEncoder = new CANcoder(ShooterConstants.hoodEncoderID);

    m_flywheelEncoder = m_flyWheelMotor1.getEncoder();

    m_turretConfig = new TalonFXConfiguration();
    m_hoodConfig = new TalonFXConfiguration();

    m_flyWheel1Config = new SparkFlexConfig();
    m_flyWheel2Config = new SparkFlexConfig();

    m_spindexerConfig = new SparkMaxConfig();
    m_indexerConfig = new SparkMaxConfig();

    m_aimingEncoderConfig = new CANcoderConfiguration();
    m_hoodEncoderConfig = new CANcoderConfiguration();

    m_aimingController = new ProfiledPIDController(ShooterConstants.aimingKP, 
                                                   ShooterConstants.aimingKI, 
                                                   ShooterConstants.aimingKD, 
                                                   ShooterConstants.aimingControllerConstraints);

    m_hoodController = new ProfiledPIDController(ShooterConstants.aimingKP, 
                                                 ShooterConstants.aimingKI, 
                                                 ShooterConstants.aimingKD, 
                                                 ShooterConstants.aimingControllerConstraints);

    m_driveTrainPositionSupplier = driveTrainPosition;
    m_driveTrainVelocitySupplier = driveTrainVelocity;
    m_driveTrainAngularVelocitySupplier = driveTrainAngularVelocity;
    m_driveTrainZoneStateSupplier = zoneState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    driveTrainPos = m_driveTrainPositionSupplier.get();
    driveTrainSpeeds = m_driveTrainVelocitySupplier.get();
    driveTrainOmega = m_driveTrainAngularVelocitySupplier.get();
    currentZone = m_driveTrainZoneStateSupplier.get();

    currentPosition = driveTrainPos.withVector(driveTrainPos.getRotation().plus(AutoAimConstants.turretOffsetPos.getXYAngle()), 
                                               AutoAimConstants.turretOffsetCoordinates.getTranslation(), 
                                               new Rotation2d());
    currentPosition3D = new Pose3d(currentPosition.getX(), currentPosition.getY(), AutoAimConstants.turretOffsetPos.getZ(), 
                                   new Rotation3d());

    canShoot = true;
    
    currentVelocity = Vector3D.getPointVelocity(new Vector3D(driveTrainSpeeds.vxMetersPerSecond, 
                                                             driveTrainSpeeds.vyMetersPerSecond), 
                                                new Vector3D(AutoAimConstants.turretOffsetCoordinates), 
                                                driveTrainOmega);                     

    aimAt(currentZone == DriveTrainZoneState.AllianceZone ? FieldConstants.hubPosition : 
          new Pose3d(currentPosition.getClosest(FieldConstants.leftShuttleTarget, FieldConstants.rightShuttleTarget)));

    if (canShoot) revFlywheel();
  }

  public void configMotorDefaults() {
    //ShooterMotors
    m_flyWheel1Config.inverted(false);
    m_flyWheel1Config.idleMode(IdleMode.kCoast);
    m_flyWheel2Config.follow(m_flyWheelMotor1, true);

    //Indexers
    m_spindexerConfig.idleMode(IdleMode.kCoast);
    m_indexerConfig.idleMode(IdleMode.kCoast);

    //TurretMotor
    m_turretConfig.Audio.BeepOnBoot = false;

    m_turretConfig.ClosedLoopGeneral.ContinuousWrap = false;
    m_turretConfig.MotorOutput.NeutralMode = ShooterConstants.turretNeutralModeValue;
    m_turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    m_turretConfig.MotorOutput.PeakForwardDutyCycle = ShooterConstants.turretMaxDutyCycle;
    m_turretConfig.MotorOutput.PeakReverseDutyCycle = -ShooterConstants.turretMaxDutyCycle;
    m_turretConfig.Voltage.PeakForwardVoltage = ShooterConstants.turretMaxVoltage;
    m_turretConfig.Voltage.PeakReverseVoltage = -ShooterConstants.turretMaxVoltage;

    //HoodMotor
    m_hoodConfig.Audio.BeepOnBoot = false;

    m_hoodConfig.ClosedLoopGeneral.ContinuousWrap = false;
    m_hoodConfig.MotorOutput.NeutralMode = ShooterConstants.hoodNeutralModeValue;
    m_hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    m_hoodConfig.MotorOutput.PeakForwardDutyCycle = ShooterConstants.hoodMaxDutyCycle;
    m_hoodConfig.MotorOutput.PeakReverseDutyCycle = -ShooterConstants.hoodMaxDutyCycle;
    m_hoodConfig.Voltage.PeakForwardVoltage = ShooterConstants.hoodMaxVoltage;
    m_hoodConfig.Voltage.PeakReverseVoltage = -ShooterConstants.hoodMaxVoltage;

    //AimingEncoder
    m_aimingEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = ShooterConstants.aimingEncoderRange;
    m_aimingEncoderConfig.MagnetSensor.SensorDirection = ShooterConstants.aimingEncoderSensorDirection;

    m_turret.getConfigurator().apply(m_turretConfig);
    m_hood.getConfigurator().apply(m_hoodConfig);
    m_aimingEncoder.getConfigurator().apply(m_aimingEncoderConfig);

    m_flyWheelMotor1.configure(m_flyWheel1Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_flyWheelMotor2.configure(m_flyWheel2Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_spindexer.configure(m_spindexerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_indexer.configure(m_indexerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void genericShoot(double speed) {
    m_flyWheelMotor1.set(speed);
  }

  public void revFlywheel() {
    genericShoot(ShooterConstants.revFlywheelSpeed);
  }

  private void runSpindexer() {
    m_spindexer.set(ShooterConstants.spindexerSpeed);
  }

  private void stopSpindexer() {
    m_spindexer.set(0);
  }

  private void runIndexer() {
    m_indexer.set(ShooterConstants.indexingSpeed);
  }

  private void stopIndexer() {
    m_indexer.set(0);
  }

  public void runIndexers() {
    runIndexer();
    runSpindexer();
  }

  public void stopIndexers() {
    stopIndexer();
    stopSpindexer();
  }

  public void aimTurret(Rotation2d desiredAngle) {
    m_turret.set(m_aimingController.calculate(m_aimingEncoder.getAbsolutePosition().getValueAsDouble() *
                                              ShooterConstants.aimingConversionFactor, 
                                              desiredAngle.getDegrees()));
  }

  public void angleHood(Rotation2d desiredAngle) {
    m_turret.set(m_hoodController.calculate(m_hoodEncoder.getAbsolutePosition().getValueAsDouble() *
                                            ShooterConstants.hoodConversionFactor, 
                                            desiredAngle.getDegrees()));
  }

  public void aimAt(Pose3d targetPose) {
    double flyWheelVelocity = m_flywheelEncoder.getVelocity();
    double horizDistance = targetPose.getX() - currentPosition.getX();
    double vertDistance = targetPose.getZ() - currentPosition3D.getZ();

    double c = -4.9 * Math.pow(horizDistance / flyWheelVelocity, 2) - vertDistance;

                              //-b             +/- sqrt(b^2
    double quadFormSolution = -horizDistance + Math.sqrt(Math.pow(horizDistance, 2) -
                                                         4 * -4.9 * Math.pow(horizDistance / flyWheelVelocity, 2) * c) / // -4ac)
                                                         -9.8 * Math.pow(horizDistance / flyWheelVelocity, 2); // /2a
    Rotation2d hoodAngle = Rotation2d.fromRadians(Math.atan(quadFormSolution));

    Vector3D turretAimVector = Vector3D.fromPoints(currentPosition3D, targetPose).minus(currentVelocity);
    Rotation2d turretAngle = Rotation2d.fromRadians(Math.atan(turretAimVector.getY() / turretAimVector.getX()));

    aimTurret(turretAngle);
    angleHood(hoodAngle);
  }
}
