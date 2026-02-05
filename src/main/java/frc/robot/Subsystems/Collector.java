package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;

public class Collector extends SubsystemBase {
  private final TalonFX m_deployMotor;
  private final SparkMax m_collectorMotor;

  private final TalonFXConfiguration m_deployMotorConfig;
  private final SparkMaxConfig m_collectorMotorConfig;

  /** Creates a new Collector. */
  public Collector() {
    m_collectorMotor = new SparkMax(CollectorConstants.collectorMotorID, MotorType.kBrushless);
    m_deployMotor = new TalonFX(CollectorConstants.deployMotorID);

    m_collectorMotorConfig = new SparkMaxConfig();
    m_deployMotorConfig = new TalonFXConfiguration();

    m_deployMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void configMotorDefaults() {
    //CollectorMotor
    m_collectorMotorConfig.idleMode(CollectorConstants.collectorMotorNeutralMode);

    //DeployMotor
    m_deployMotorConfig.Audio.BeepOnBoot = false;

    m_deployMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;
    m_deployMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    m_deployMotorConfig.MotorOutput.PeakForwardDutyCycle = CollectorConstants.deployMotorMaxDutyCycle;
    m_deployMotorConfig.MotorOutput.PeakReverseDutyCycle = -CollectorConstants.deployMotorMaxDutyCycle;

    m_deployMotorConfig.Voltage.PeakForwardVoltage = CollectorConstants.deployMotorMaxVoltage;
    m_deployMotorConfig.Voltage.PeakReverseVoltage = -CollectorConstants.deployMotorMaxVoltage;

    m_deployMotorConfig.Slot0.kP = CollectorConstants.deployKP;
    m_deployMotorConfig.Slot0.kI = CollectorConstants.deployKI;
    m_deployMotorConfig.Slot0.kD = CollectorConstants.deployKD;

    m_collectorMotor.configure(m_collectorMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_deployMotor.getConfigurator().apply(m_deployMotorConfig);
  }

  public void collect() {
    m_collectorMotor.set(CollectorConstants.collectionSpeed);
  }

  public void stopCollector() {
    m_collectorMotor.set(0);
  }
}
