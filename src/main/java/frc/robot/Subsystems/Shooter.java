package frc.robot.Subsystems;

import java.util.Vector;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Lib.AdvancedPose2D;
import frc.Lib.BlazeMath;
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

  private final RelativeEncoder m_flywheelEncoder;

  private final Field2d simField;

  private final PIDController m_bigMoveAimingController, m_fineTuneAimingController, m_hoodController;
  private final SparkClosedLoopController m_velocityController;

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
  private Rotation2d turretAngle, hoodAngle;

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

    m_flywheelEncoder = m_flyWheelMotor1.getEncoder();

    m_turretConfig = new TalonFXConfiguration();
    m_hoodConfig = new TalonFXConfiguration();

    m_flyWheel1Config = new SparkFlexConfig();
    m_flyWheel2Config = new SparkFlexConfig();

    m_spindexerConfig = new SparkMaxConfig();
    m_indexerConfig = new SparkMaxConfig();

    m_bigMoveAimingController = new PIDController(ShooterConstants.bigMoveAimingKP, 
                                                  ShooterConstants.bigMoveAimingKI, 
                                                  ShooterConstants.bigMoveAimingKD);

    m_fineTuneAimingController = new PIDController(ShooterConstants.fineTuneAimingKP, 
                                                   ShooterConstants.finetuneAimingKI, 
                                                   ShooterConstants.fineTuneAimingKD);
                                                   
    m_bigMoveAimingController.disableContinuousInput();
    m_fineTuneAimingController.disableContinuousInput();

    m_hoodController = new PIDController(ShooterConstants.hoodKP, 
                                         ShooterConstants.hoodKI, 
                                         ShooterConstants.hoodKD);

    m_velocityController = m_flyWheelMotor1.getClosedLoopController();

    m_driveTrainPositionSupplier = driveTrainPosition;
    m_driveTrainVelocitySupplier = driveTrainVelocity;
    m_driveTrainAngularVelocitySupplier = driveTrainAngularVelocity;
    m_driveTrainZoneStateSupplier = zoneState;

    configMotorDefaults();

    //m_hood.setPosition(0);
    // m_turret.setPosition(0);
    hoodAngle = Rotation2d.fromDegrees(m_hood.getPosition().getValueAsDouble() * ShooterConstants.hoodConversionFactor);
    turretAngle = Rotation2d.fromDegrees(m_turret.getPosition().getValueAsDouble() * ShooterConstants.aimingConversionFactor);

    simField = new Field2d();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    driveTrainPos = m_driveTrainPositionSupplier.get();
    driveTrainSpeeds = m_driveTrainVelocitySupplier.get();
    driveTrainOmega = m_driveTrainAngularVelocitySupplier.get();
    currentZone = m_driveTrainZoneStateSupplier.get();

    // driveTrainSpeeds = new ChassisSpeeds(3, 1, Math.PI/4);
    // driveTrainOmega = new Vector3D(0, 0, -driveTrainSpeeds.omegaRadiansPerSecond);

    Vector3D driveTrainVelocityVector = new Vector3D(driveTrainSpeeds.vxMetersPerSecond, driveTrainSpeeds.vyMetersPerSecond);

    currentPosition = driveTrainPos.withVector(driveTrainPos.getRotation(), 
                                               AutoAimConstants.turretOffsetCoordinates.getTranslation(), 
                                               new Rotation2d());
    currentPosition3D = new Pose3d(currentPosition.getX(), currentPosition.getY(), AutoAimConstants.turretOffsetPos.getZ(), 
                                   new Rotation3d());

    Vector3D vectorToTrench = Vector3D.fromPoints(driveTrainPos, driveTrainPos.getClosest(
                                                                          FieldConstants.leftTrench.getMidpoint(), 
                                                                          FieldConstants.rightTrench.getMidpoint()));
    canShoot = true;
    // !FieldConstants.trenchZone.pointInZone(driveTrainPos) &&
    //            vectorToTrench.getDotProduct(driveTrainVelocityVector) < 
    //               AutoAimConstants.dotProductThreshold * Math.pow(vectorToTrench.get2DMagnitude(), 2);
    
    Vector3D currentVelocity2D = Vector3D.getPointVelocity(new Vector3D(driveTrainSpeeds.vxMetersPerSecond, 
                                                             driveTrainSpeeds.vyMetersPerSecond), 
                                                new Vector3D(currentPosition), 
                                                driveTrainOmega);   
                                                
    currentVelocity = new Vector3D(new AdvancedPose2D().withVector(driveTrainPos.getRotation(), 
        new Translation2d(currentVelocity2D.getX(), currentVelocity2D.getY()), new Rotation2d()));
                                                
    Pose3d targetPose = currentZone == DriveTrainZoneState.AllianceZone ? FieldConstants.hubPosition : 
                          new Pose3d(currentPosition.getClosest(FieldConstants.leftShuttleTarget, 
                                                                FieldConstants.rightShuttleTarget));

    if (canShoot) {
      //revFlywheel();
      //aimAt(targetPose);
    } else {
      hoodAngle = Rotation2d.fromDegrees(ShooterConstants.minHoodHeight);
    }

    // Pose3d targetPose = new Pose3d(5,5,0, new Rotation3d());
    Vector3D turretAimVector = Vector3D.fromPoints(currentPosition3D, targetPose).minus(currentVelocity);
    turretAngle = Rotation2d.fromRadians(Math.atan2(turretAimVector.getY(), turretAimVector.getX())).plus(driveTrainPos.getRotation());
    aimTurret(turretAngle);
    // angleHood(hoodAngle);

    SmartDashboard.putNumber("Turret Pos", getTurretPos());
    SmartDashboard.putNumber("HoodPos", getHoodPos());
    SmartDashboard.putNumber("DesTurAngle", turretAngle.getDegrees());

    // aimTurret(Rotation2d.fromDegrees(0));
    //revFlywheel();
  }

  public void configMotorDefaults() {
    //ShooterMotors
    m_flyWheel1Config.inverted(true);
    m_flyWheel1Config.idleMode(IdleMode.kCoast);
    m_flyWheel2Config.follow(m_flyWheelMotor1, true);

    //Indexers
    m_spindexerConfig.idleMode(IdleMode.kCoast);
    m_indexerConfig.idleMode(IdleMode.kCoast);

    m_spindexerConfig.inverted(false);
    m_indexerConfig.inverted(true);

    //TurretMotor
    m_turretConfig.Audio.BeepOnBoot = false;

    m_turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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

    //Apply Configs
    m_turret.getConfigurator().apply(m_turretConfig);
    m_hood.getConfigurator().apply(m_hoodConfig);

    m_flyWheelMotor1.configure(m_flyWheel1Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_flyWheelMotor2.configure(m_flyWheel2Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_spindexer.configure(m_spindexerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_indexer.configure(m_indexerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void genericShoot(double speed) {
    m_flyWheelMotor1.set(speed);
  }

  public void revFlywheel() {
    double velocity = .4; //ShooterConstants.maxVelocity;
                      // ShooterConstants.minVelocity + 
                      // (ShooterConstants.maxVelocity - ShooterConstants.minVelocity) *
                      // (driveTrainPos.getDistance(FieldConstants.hubCoordinates) / FieldConstants.outpostPos
                      //                                                               .getDistance(FieldConstants.hubCoordinates));

    genericShoot(velocity + ShooterConstants.velocityAddOn);
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

  public double getTurretPos() {
    return m_turret.getPosition().getValueAsDouble() * ShooterConstants.aimingConversionFactor + ShooterConstants.turretAngleOffset;
  }

  public double getRawTurretPos() {
    return m_turret.getPosition().getValueAsDouble();
  }

  public double getHoodPos() {
    return m_hood.getPosition().getValueAsDouble() * ShooterConstants.hoodConversionFactor;
  }

  public double getRawHoodPos() {
    return m_hood.getPosition().getValueAsDouble();
  }

  public void aimTurret(Rotation2d desiredAngle) {
    double target = desiredAngle.getDegrees();
    target = MathUtil.inputModulus(target, -45, 315);
    m_turret.set(BlazeMath.clampMagnitude(Math.abs(getTurretPos() - target) > ShooterConstants.moveTypeThreshold
                                            ? m_bigMoveAimingController.calculate(getTurretPos(), target) 
                                            : m_fineTuneAimingController.calculate(getTurretPos(), target),
                                ShooterConstants.maxTurretOutput));

    SmartDashboard.putBoolean("Big", Math.abs(getTurretPos() - target) > ShooterConstants.moveTypeThreshold);

    SmartDashboard.putNumber("Aiming Factor", BlazeMath.clampMagnitude((Math.abs(getTurretPos() - target) > ShooterConstants.moveTypeThreshold
                                            ? m_bigMoveAimingController.calculate(getTurretPos(), target) 
                                            : m_fineTuneAimingController.calculate(getTurretPos(), target)),
                                ShooterConstants.maxTurretOutput));

    SmartDashboard.putNumber("FilteredDesTurPos", target);
  }

  public void angleHood(Rotation2d desiredAngle) {
    double target = 90 - desiredAngle.getDegrees();

    m_hood.set(m_hoodController.calculate(getHoodPos(), target));
  }

  public void aimAt(Pose3d targetPose) {
    targetPose = new Pose3d(0, 0, 0, new Rotation3d());
    double flyWheelVelocity = 5; //m_flywheelEncoder.getVelocity();
    double horizDistance = targetPose.getX() - currentPosition.getX();
    double vertDistance = targetPose.getZ() - currentPosition3D.getZ();

    double a = -4.9 * Math.pow(horizDistance / flyWheelVelocity, 2);
    double b = horizDistance;
    double c = -(4.9 * Math.pow(horizDistance / flyWheelVelocity, 2) + vertDistance);
    double[] abc = {a, b, c};
    SmartDashboard.putNumberArray("ABC", abc);

    double quadFormSolution = (-b - Math.sqrt(Math.pow(b, 2) - 4 * a * c)) / (2 * a);
    SmartDashboard.putNumber("QFS", quadFormSolution);

    //hoodAngle = Rotation2d.fromRadians(Math.atan(quadFormSolution));
    SmartDashboard.putNumber("hood angle", hoodAngle.getDegrees());

    double t = horizDistance / (flyWheelVelocity * hoodAngle.getCos());
    SmartDashboard.putNumber("solved y", flyWheelVelocity * hoodAngle.getSin() * t - 4.9 * Math.pow(t, 2));
  }

  public Command runIndex() {
    return Commands.runEnd(() -> this.runIndexers(), 
                           () -> this.stopIndexers());
  }
}
