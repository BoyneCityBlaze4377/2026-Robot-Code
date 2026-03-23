package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;

public class Collector extends SubsystemBase {
  private final TalonFX m_deployMotor;
  private final SparkMax m_collectorMotor;

  private final TalonFXConfiguration m_deployMotorConfig;
  private final SparkMaxConfig m_collectorMotorConfig;

  private final PIDController m_deployController;

  private final Timer m_timer = new Timer();

  private double setpoint, jostleSpeed = 0;
  private boolean isJostling = false;

  /** Creates a new Collector. */
  public Collector() {
    m_collectorMotor = new SparkMax(CollectorConstants.collectorMotorID, MotorType.kBrushless);
    m_deployMotor = new TalonFX(CollectorConstants.deployMotorID);

    m_collectorMotorConfig = new SparkMaxConfig();
    m_deployMotorConfig = new TalonFXConfiguration();

    m_deployController = new PIDController(CollectorConstants.deployKP, 
                                           CollectorConstants.deployKI, 
                                           CollectorConstants.deployKD);

    setpoint = getCollectorPos();

    m_deployMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // moveCollector();
    SmartDashboard.putNumber("DeployMotor Pos", m_deployMotor.getPosition().getValueAsDouble() 
    * CollectorConstants.deployConversionFactor);
    SmartDashboard.putNumber("DeploySP", getCollectorPos());
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

    //Load Configs
    m_collectorMotor.configure(m_collectorMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_deployMotor.getConfigurator().apply(m_deployMotorConfig);
  }

  public void moveCollector() {
    m_deployMotor.set(MathUtil.clamp(m_deployController.calculate(getCollectorPos(), setpoint), 
                                     -CollectorConstants.maxDeploySpeed, CollectorConstants.maxDeploySpeed)
                      + jostleSpeed);

    SmartDashboard.putNumber("Deploy Factor", MathUtil.clamp(m_deployController.calculate(getCollectorPos(), setpoint), 
                                     -CollectorConstants.maxDeploySpeed, CollectorConstants.maxDeploySpeed)
                      + jostleSpeed);
  }

  public double getCollectorPos() {
    return m_deployMotor.getPosition().getValueAsDouble() * CollectorConstants.deployConversionFactor;
  }

  public double getJostleSpeed(double time) {
    isJostling = true;
    int intervalID = (int) Math.floor(time / CollectorConstants.jostleInterval) % 2;
    double timeBasedSpeed = (intervalID < 1e-6 ? CollectorConstants.jostleSpeedDifferential : 
                                                -CollectorConstants.jostleSpeedDifferential);
    return (!(CollectorConstants.deployedPos - getPosition() < CollectorConstants.closeToExtendedTolerance) 
                      ? timeBasedSpeed : -CollectorConstants.jostleSpeedDifferential);
  }

  public void stopJostle() {
    jostleSpeed = 0;
    isJostling = false;
  }

  public void setSetpoint(double target) {
    setpoint = target;
  }

  public void collect() {
    m_collectorMotor.set(CollectorConstants.collectionSpeed);
  }

  public void stopCollector() {
    m_collectorMotor.set(0);
  }

  public double getPosition() {
    return m_deployMotor.getPosition().getValueAsDouble();
  }

  public Command runCollector() {
    return Commands.runEnd(() -> this.collect(), 
                           () -> this.stopCollector());
  }

  public Command deployCollector() {
    return Commands.runOnce(() -> this.setSetpoint(CollectorConstants.deployedPos - CollectorConstants.setpointOffset), this);
  }

  public Command retractCollector() {
    return Commands.runOnce(() -> this.setSetpoint(CollectorConstants.retractedPos + CollectorConstants.setpointOffset), this);
  }

  public Command Collect() {
    return Commands.parallel(this.deployCollector(), this.runCollector() 
    //, this.Jostle()
    );
  }

  public Command Jostle() {
    return Commands.runEnd(() -> {
      if (!isJostling) m_timer.restart();
      jostleSpeed = getJostleSpeed(m_timer.get());
    }, () -> {
      stopJostle();
    });
  }
}
