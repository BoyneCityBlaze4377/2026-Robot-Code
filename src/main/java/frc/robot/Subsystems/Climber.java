package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ShooterConstants;

public class Climber extends SubsystemBase {
  private final TalonFX m_climberMotor;
  private final TalonFXConfiguration m_climberMotorConfig;
  
  private final CANcoder m_climberEncoder;
  private final CANcoderConfiguration m_climberEncoderConfig;

  /** Creates a new Climber. */
  public Climber() {
    m_climberMotor = new TalonFX(ClimberConstants.climberMotorID);
    m_climberEncoder = new CANcoder(ClimberConstants.climberEncoderID);

    m_climberMotorConfig = new TalonFXConfiguration();
    m_climberEncoderConfig = new CANcoderConfiguration();

    configDefaults();
  }

  public void configDefaults() {
    //Motor
    m_climberMotorConfig.Audio.BeepOnBoot = false;

    m_climberMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_climberMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;

    m_climberMotorConfig.MotorOutput.PeakForwardDutyCycle = ClimberConstants.climberMaxDutyCycle;
    m_climberMotorConfig.MotorOutput.PeakReverseDutyCycle = -ClimberConstants.climberMaxDutyCycle;
    m_climberMotorConfig.Voltage.PeakForwardVoltage = ClimberConstants.climberMaxVoltage;
    m_climberMotorConfig.Voltage.PeakReverseVoltage = -ClimberConstants.climberMaxVoltage;

    //Encoder
    m_climberEncoderConfig.MagnetSensor.SensorDirection = ClimberConstants.climberEncoderSensorDirection;

    m_climberMotor.getConfigurator().apply(m_climberMotorConfig);
    m_climberEncoder.getConfigurator().apply(m_climberEncoderConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
