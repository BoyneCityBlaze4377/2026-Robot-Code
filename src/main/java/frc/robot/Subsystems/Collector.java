package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;

public class Collector extends SubsystemBase {
  private final TalonFX m_collectorMotor, m_deployMotor;

  private final TalonFXConfiguration m_collectorMotorConfig, m_deployMotorConfig;
  /** Creates a new Collector. */
  public Collector() {
    m_collectorMotor = new TalonFX(CollectorConstants.collectorMotorID);
    m_deployMotor = new TalonFX(CollectorConstants.deployMotorID);

    m_collectorMotorConfig = new TalonFXConfiguration();
    m_deployMotorConfig = new TalonFXConfiguration();

    m_deployMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void configMotorDefaults() {
    //CollectorMotor
    m_collectorMotorConfig.Audio.BeepOnBoot = false;

    m_collectorMotorConfig.MotorOutput.NeutralMode = CollectorConstants.collectorMotorNeutralMode;

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

    m_collectorMotor.getConfigurator().apply(m_collectorMotorConfig);
    m_deployMotor.getConfigurator().apply(m_deployMotorConfig);
  }
}
