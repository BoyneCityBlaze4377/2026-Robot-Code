package frc.robot.Collector;

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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;

public class Collector extends SubsystemBase {
  public enum CollectorMode {COLLECTING, JOSTLING, RETRACTED};

  private final TalonFX m_deployMotor;
  private final SparkMax m_collectorMotor;

  private final TalonFXConfiguration m_deployMotorConfig;
  private final SparkMaxConfig m_collectorMotorConfig;

  private final PIDController m_deployController;

  public final Debouncer resistanceDebouncer = new Debouncer(CollectorConstants.resistanceDebounceTime, DebounceType.kBoth);

  private final Timer m_timer = new Timer();

  private double setpoint, jostlePosDiff = 0;
  private boolean jostlingEnabled = false;

  public CollectorState m_desiredState, m_currentState;
  public CollectorMode currentMode = CollectorMode.RETRACTED;

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

    m_deployMotor.setPosition(CollectorConstants.retractedPos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    switch (currentMode) {
      case RETRACTED:
        setDesiredState(CollectorState.fullyRetracted);
        jostlingEnabled = false;
        break;
      case COLLECTING:
        setDesiredState(CollectorState.collecting);
        jostlingEnabled = false;
        break;
      case JOSTLING:
        setDesiredState(CollectorState.fullyRetracted);
        jostlingEnabled = true;
        break;
      default:
        setDesiredState(CollectorState.fullyRetracted);
        jostlingEnabled = false;
    }

    setpoint = m_desiredState.position;
    moveCollector(isResisted(jostlingEnabled));
    runHarvester(m_desiredState.velocity);

    m_currentState = new CollectorState(getCollectorPos(), m_collectorMotor.get());
    SmartDashboard.putString("CollectorMode", getCurrentMode().toString());

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

  public void moveCollector(boolean isResisted) {
    double PIDSpeed = MathUtil.clamp(m_deployController.calculate(getCollectorPos(), setpoint + jostlePosDiff), 
                                     -CollectorConstants.maxDeploySpeed, CollectorConstants.maxDeploySpeed);
    m_deployMotor.set(isResisted ? CollectorConstants.resistanceSpeed * Math.signum(PIDSpeed) : PIDSpeed);

    SmartDashboard.putNumber("Deploy Factor", MathUtil.clamp(m_deployController.calculate(getCollectorPos(), setpoint), 
                                     -CollectorConstants.maxDeploySpeed, CollectorConstants.maxDeploySpeed));
  }

  public double getCollectorPos() {
    return m_deployMotor.getPosition().getValueAsDouble() * CollectorConstants.deployConversionFactor;
  }

  public CollectorMode getCurrentMode() {
    return currentMode;
  }

  // public double getJostleSpeed(double time) {
  //   isJostling = true;
  //   int intervalID = (int) Math.floor(time / CollectorConstants.jostleInterval) % 2;
  //   double timeBasedSpeed = (intervalID < 1e-6 ? CollectorConstants.jostlePositionDifferential : 
  //                                               -CollectorConstants.jostlePositionDifferential);
  //   return (!(CollectorConstants.deployedPos - getPosition() < CollectorConstants.closeToExtendedTolerance) 
  //                     ? timeBasedSpeed : -CollectorConstants.jostlePositionDifferential);
  // }

  public void setDesiredState(CollectorState desiredState) {
    m_desiredState = desiredState;
  }

  public CollectorState getCurrentState() {
    return m_currentState;
  }

  public boolean isResisted(boolean jostleEnabled) {
    return jostleEnabled && resistanceDebouncer.calculate(currentMode == CollectorMode.JOSTLING && 
                                                          m_deployMotor.getVelocity().getValueAsDouble() 
                                                              <= CollectorConstants.restistanceSpeedthreshold);
  }

  public void setCurrentMode(CollectorMode newMode) {
    currentMode = newMode;
  }

  public void setSetpoint(double target) {
    setpoint = target;
  }

  public void runHarvester(double speed) {
    m_collectorMotor.set(speed);
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

  public Command SetCollectorMode(CollectorMode mode) {
    return Commands.runOnce(() -> this.setCurrentMode(mode), this);
  }

  // public Command Jostle() {
    // return Commands.runEnd(() -> {
    //   if (!isJostling) m_timer.restart();
    //   jostleSpeed = getJostleSpeed(m_timer.get());
    // }, () -> {
    //   stopJostle();
    // });
  // }
}
