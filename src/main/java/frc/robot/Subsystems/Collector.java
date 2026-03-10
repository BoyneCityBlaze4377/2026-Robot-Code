// package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;

public class Collector extends SubsystemBase {
  private final TalonFX m_deployMotor;
  private final SparkMax m_collectorMotor;

  private final CANcoder m_deployEncoder;

  private final TalonFXConfiguration m_deployMotorConfig;
  private final SparkMaxConfig m_collectorMotorConfig;

  private final CANcoderConfiguration m_deployEncoderConfig;

  private final PIDController m_deployController;

  private double setpoint;

  /** Creates a new Collector. */
  public Collector() {
    m_collectorMotor = new SparkMax(CollectorConstants.collectorMotorID, MotorType.kBrushless);
    m_deployMotor = new TalonFX(CollectorConstants.deployMotorID);

    m_deployEncoder = new CANcoder(CollectorConstants.deployEncoderID);

//     m_collectorMotorConfig = new SparkMaxConfig();
//     m_deployMotorConfig = new TalonFXConfiguration();

    m_deployEncoderConfig = new CANcoderConfiguration();

    m_deployController = new PIDController(CollectorConstants.deployKP, 
                                           CollectorConstants.deployKI, 
                                           CollectorConstants.deployKD);

    setpoint = m_deployEncoder.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    moveCollector();
  }

//   public void configMotorDefaults() {
//     //CollectorMotor
//     m_collectorMotorConfig.idleMode(CollectorConstants.collectorMotorNeutralMode);

//     //DeployMotor
//     m_deployMotorConfig.Audio.BeepOnBoot = false;

//     m_deployMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;
//     m_deployMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

//     m_deployMotorConfig.MotorOutput.PeakForwardDutyCycle = CollectorConstants.deployMotorMaxDutyCycle;
//     m_deployMotorConfig.MotorOutput.PeakReverseDutyCycle = -CollectorConstants.deployMotorMaxDutyCycle;

//     m_deployMotorConfig.Voltage.PeakForwardVoltage = CollectorConstants.deployMotorMaxVoltage;
//     m_deployMotorConfig.Voltage.PeakReverseVoltage = -CollectorConstants.deployMotorMaxVoltage;

//     m_deployMotorConfig.Slot0.kP = CollectorConstants.deployKP;
//     m_deployMotorConfig.Slot0.kI = CollectorConstants.deployKI;
//     m_deployMotorConfig.Slot0.kD = CollectorConstants.deployKD;

    //DeployEncoder
    m_deployEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = CollectorConstants.encoderRange;
    m_deployEncoderConfig.MagnetSensor.SensorDirection = CollectorConstants.encoderDirection;

    //Load Configs
    m_collectorMotor.configure(m_collectorMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_deployMotor.getConfigurator().apply(m_deployMotorConfig);
    m_deployEncoder.getConfigurator().apply(m_deployEncoderConfig);
  }

  public void moveCollector() {
    m_deployMotor.set(m_deployController.calculate(m_deployEncoder.getPosition().getValueAsDouble(), setpoint));
  }

  public void setSetpoint(double target) {
    setpoint = target;
  }

//   public void collect() {
//     m_collectorMotor.set(CollectorConstants.collectionSpeed);
//   }

//   public void stopCollector() {
//     m_collectorMotor.set(0);
//   }

  private Command runCollector() {
    return Commands.runEnd(() -> this.collect(), 
                           () -> this.stopCollector());
  }

  private Command deployCollector() {
    return Commands.runOnce(() -> this.setSetpoint(CollectorConstants.deployedPos), this);
  }

  private Command retractCollector() {
    return Commands.runOnce(() -> this.setSetpoint(CollectorConstants.retractedPos), this);
  }

  public Command Collect() {
    return Commands.parallel(this.deployCollector(), this.runCollector());
  }
}
