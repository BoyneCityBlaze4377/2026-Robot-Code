package frc.robot.Shooter;

import java.util.function.BooleanSupplier;
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
import frc.robot.DriveTrain.DriveTrain;
import frc.robot.DriveTrain.DriveTrain.DriveTrainZoneState;

public class Shooter extends SubsystemBase {
  public enum ShooterMode {SHOOTING, IDLE, TRENCH, AIMING};

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

  public ShooterState m_desiredState, m_currentState;
  public ShooterMode currentMode = ShooterMode.IDLE, desiredMode = ShooterMode.IDLE;

  public boolean isCollecting = false;

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

    m_hood.setPosition(ShooterConstants.minHoodHeight / ShooterConstants.hoodConversionFactor);
    // m_turret.setPosition(0);

    simField = new Field2d();

    m_desiredState = new ShooterState(Rotation2d.fromDegrees(getTurretPos()));
    m_currentState = new ShooterState(Rotation2d.fromDegrees(getTurretPos()), 
                                      Rotation2d.fromDegrees(90 - getHoodPos()), 
                                      getVelocity(),
                                      m_desiredState.isShooting);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    driveTrainPos = m_driveTrainPositionSupplier.get();
    driveTrainSpeeds = m_driveTrainVelocitySupplier.get();
    driveTrainOmega = m_driveTrainAngularVelocitySupplier.get();
    currentZone = m_driveTrainZoneStateSupplier.get();

    AdvancedPose2D closestTrench = driveTrainPos.getClosest(FieldConstants.leftTrench.getMidpoint(), 
                                                            FieldConstants.rightTrench.getMidpoint());

    boolean trenchFlag = ShooterMath.tooCloseToTrench(driveTrainPos, closestTrench) &&
                         ShooterMath.movingTowardsTrench(driveTrainPos, driveTrainSpeeds, closestTrench);

    currentPosition = driveTrainPos.withVector(driveTrainPos.getRotation(), 
                                                AutoAimConstants.turretOffsetCoordinates.getTranslation(), 
                                                new Rotation2d());
                                                
    Pose3d targetPose = currentZone == DriveTrainZoneState.AllianceZone ? FieldConstants.hubPosition : 
                          new Pose3d(currentPosition.getClosest(FieldConstants.leftShuttleTarget, 
                                                                FieldConstants.rightShuttleTarget));

    ShooterState stateToTarget = ShooterMath.calcDesiredState(driveTrainPos, 
                                                              driveTrainSpeeds, 
                                                              driveTrainOmega, 
                                                              targetPose, 
                                                              getVelocity());
                                                   
    if (desiredMode == ShooterMode.SHOOTING) {
      if (ShooterMath.moveIsTooBig(getTurretPos(), m_desiredState.angleToTarget.getDegrees())) {
        //currentMode = ShooterMode.AIMING;
        currentMode = ShooterMode.SHOOTING;
      } else if (trenchFlag) {
        currentMode = ShooterMode.TRENCH;
      } else {
        currentMode = ShooterMode.SHOOTING;
      }
    } else {
      currentMode = desiredMode;
    }
    
    if (currentMode == ShooterMode.IDLE && isCollecting) currentMode = ShooterMode.AIMING;
    if (currentMode == ShooterMode.AIMING && trenchFlag) currentMode = ShooterMode.TRENCH;

    switch (currentMode) {
      case IDLE:
        setDesiredState(new ShooterState(stateToTarget.angleToTarget).noShooting());
        break;
      case SHOOTING:
        setDesiredState(stateToTarget);
        break;
      case AIMING:
        setDesiredState(stateToTarget.noShooting());
        break;
      case TRENCH:
        setDesiredState(new ShooterState(stateToTarget.angleToTarget, 
                                         Rotation2d.fromDegrees(90 - ShooterConstants.minHoodHeight), 
                                         stateToTarget.shotVelocity,
                                         false));
        break;
      default:
        setDesiredState(new ShooterState(stateToTarget.angleToTarget).noShooting());
        break;
    }

    // aimTurret(m_desiredState.angleToTarget);
    // angleHood(m_desiredState.shotPitch);
    revFlywheel(m_desiredState.shotVelocity);
    runIndexers(m_desiredState.isShooting);

    AdvancedPose2D currentPosition = driveTrainPos.withVector(driveTrainPos.getRotation(), 
                                                AutoAimConstants.turretOffsetCoordinates.getTranslation(), 
                                                new Rotation2d());

    simField.setRobotPose(driveTrainPos);

    m_currentState = new ShooterState(Rotation2d.fromDegrees(getTurretPos()), 
                                      Rotation2d.fromDegrees(90 - getHoodPos()), 
                                      getVelocity(),
                                      m_desiredState.isShooting);

    SmartDashboard.putString("ShooterMode", currentMode.toString());
    SmartDashboard.putString("Shooter Desired State", m_desiredState.toString());
    SmartDashboard.putString("ShooterStateToTarget", stateToTarget.toString());

    simField.getObject("Turret pos").setPose(currentPosition.withRotation(m_desiredState.angleToTarget));
    simField.getObject("Target").setPose(targetPose.toPose2d());
    SmartDashboard.putData("shoot field",simField);

    SmartDashboard.putNumber("Turret Pos", getTurretPos());
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

  public void revFlywheel(double shotVelocity) {
    // double targetVelocity = ShooterMath.getMotorOutputFromVelocity(shotVelocity);
    // double velocity = targetVelocity < 0 ? 0 : targetVelocity;
    genericShoot(shotVelocity//velocity + ShooterConstants.velocityAddOn
    );
    // SmartDashboard.putNumber("vel", velocity);
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

  public void runIndexers(boolean run) {
    if (run) {
      runIndexer();
      runSpindexer();
    } else {
      stopIndexer();
      stopSpindexer();
    }
  }

  public double getTurretPos() {
    return m_turret.getPosition().getValueAsDouble() * ShooterConstants.aimingConversionFactor 
                  + ShooterConstants.turretAngleOffset;
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
    double target = MathUtil.inputModulus(desiredAngle.getDegrees(), 0, 360);
    if (!BlazeMath.withinRange(target, 90, 270, true)) target = 180;

    m_turret.set(BlazeMath.clampMagnitude(Math.abs(getTurretPos() - target) > ShooterConstants.moveTypeThreshold
                                            ? m_bigMoveAimingController.calculate(getTurretPos(), target) 
                                            : m_fineTuneAimingController.calculate(getTurretPos(), target),
                                ShooterConstants.maxTurretOutput));

    SmartDashboard.putNumber("Aiming Factor", BlazeMath.clampMagnitude((Math.abs(getTurretPos() - target) > ShooterConstants.moveTypeThreshold
                                            ? m_bigMoveAimingController.calculate(getTurretPos(), target) 
                                            : m_fineTuneAimingController.calculate(getTurretPos(), target)),
                                ShooterConstants.maxTurretOutput));

    SmartDashboard.putNumber("FilteredDesTurPos", target);
  }

  public void angleHood(Rotation2d desiredAngle) {
    double target = 90 - desiredAngle.getDegrees();

    //m_hood.set(m_hoodController.calculate(getHoodPos(), target));
    SmartDashboard.putNumber("HoodTarget", target);
    SmartDashboard.putNumber("hoodpos", getHoodPos());
  }

  public double getVelocity() {
    return ShooterMath.getVelocityFromMotorOutput(m_flyWheelMotor1.get());
    //Units.rotationsPerMinuteToRadiansPerSecond(m_flywheelEncoder.getVelocity()) * Units.inchesToMeters(2);
  }

  public void setDesiredState(ShooterState desiredState) {
    m_desiredState = desiredState;
  }

  public ShooterState getCurrentState() {
    return m_currentState;
  }

  public void setDesiredMode(ShooterMode newMode) {
    desiredMode = newMode;
  }

  public ShooterMode getCurrentMode() {
    return currentMode;
  }

  public void setIsCollecting(BooleanSupplier collecting) {
    isCollecting = collecting.getAsBoolean();
  }

  public Command SetShooterMode(ShooterMode mode) {
    return Commands.runOnce(() -> this.setDesiredMode(mode));
  }
}
