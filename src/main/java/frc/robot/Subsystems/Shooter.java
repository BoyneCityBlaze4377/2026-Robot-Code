package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final TalonFX m_shooter, m_indexer, m_turret, m_hood;
  private final TalonFXConfiguration m_shooterConfig, m_indexerConfig, m_turretConfig, m_hoodConfig;

  private final CANcoder m_aimingEncoder;
  private final CANcoderConfiguration m_aimingEncoderConfig;

  private final ProfiledPIDController m_aimingController, m_hoodController;

  private boolean INTEROP_TESTING;

  /** Creates a new Shooter. */
  public Shooter() {
    m_shooter = new TalonFX(ShooterConstants.shooterMotorID);
    m_indexer = new TalonFX(ShooterConstants.indexerMotorID);
    m_turret = new TalonFX(ShooterConstants.turretMotorID);
    m_hood = new TalonFX(ShooterConstants.hoodMotorID);

    m_aimingEncoder = new CANcoder(ShooterConstants.aimingEncoderID);

    m_shooterConfig = new TalonFXConfiguration();
    m_indexerConfig = new TalonFXConfiguration();
    m_turretConfig = new TalonFXConfiguration();
    m_hoodConfig = new TalonFXConfiguration();

    m_aimingEncoderConfig = new CANcoderConfiguration();

    m_aimingController = new ProfiledPIDController(ShooterConstants.aimingKP, 
                                                   ShooterConstants.aimingKI, 
                                                   ShooterConstants.aimingKD, 
                                                   ShooterConstants.aimingControllerConstraints);

    m_hoodController = new ProfiledPIDController(ShooterConstants.aimingKP, 
                                                 ShooterConstants.aimingKI, 
                                                 ShooterConstants.aimingKD, 
                                                 ShooterConstants.aimingControllerConstraints);

    m_shooter.setPosition(0);

    INTEROP_TESTING = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean getTestParam() {
    return INTEROP_TESTING;
  }

  public void configMotorDefaults() {
    //ShooterMotor
    m_shooterConfig.Audio.BeepOnBoot = false;

    m_shooterConfig.ClosedLoopGeneral.ContinuousWrap = false;
    m_shooterConfig.MotorOutput.NeutralMode = ShooterConstants.shootingMotorNeutralMode;
    m_shooterConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    m_shooterConfig.MotorOutput.PeakForwardDutyCycle = ShooterConstants.shooterMaxDutyCycle;
    m_shooterConfig.Voltage.PeakForwardVoltage = ShooterConstants.shooterMaxVoltage;

    m_shooterConfig.Slot0.kP = ShooterConstants.shootingKP;
    m_shooterConfig.Slot0.kI = ShooterConstants.shootingKI;
    m_shooterConfig.Slot0.kD = ShooterConstants.shootingKD;

    //IndexerMotor
    m_indexerConfig.Audio.BeepOnBoot = false;

    m_indexerConfig.MotorOutput.NeutralMode = ShooterConstants.indexingNeutralModeValue;

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

    m_shooter.getConfigurator().apply(m_shooterConfig);
    m_indexer.getConfigurator().apply(m_indexerConfig);
    m_turret.getConfigurator().apply(m_turretConfig);
    m_hood.getConfigurator().apply(m_hoodConfig);
    m_aimingEncoder.getConfigurator().apply(m_aimingEncoderConfig);
  }

  public void genericShoot(double speed) {
    m_shooter.set(speed);
  }

  public void velocityshoot(double velocity) { 
    m_shooter.setControl(new VelocityVoltage(velocity).withSlot(0));
  }

  public void index() {
    m_indexer.set(ShooterConstants.indexingSpeed);
  }

  public void stopIndex() {
    m_indexer.set(0);
  }

  public void aimAt(Rotation2d desiredAngle) {
    m_turret.set(m_aimingController.calculate(m_aimingEncoder.getAbsolutePosition().getValueAsDouble() 
                                                  * ShooterConstants.aimingConversionFactor, 
                                              desiredAngle.getDegrees()));
  }

  public boolean PARAM() {
    return INTEROP_TESTING;
  }
}
