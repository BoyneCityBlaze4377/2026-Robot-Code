package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final TalonFX m_climberMotor;
  private final TalonFXConfiguration m_climberMotorConfig;

  private final PIDController m_climberController;
  
  /** Creates a new Climber. */
  public Climber() {
    m_climberMotor = new TalonFX(ClimberConstants.climberMotorID);
    m_climberMotorConfig = new TalonFXConfiguration();

    m_climberController = new PIDController(ClimberConstants.climberKP, 
                                            ClimberConstants.climberKI, 
                                            ClimberConstants.climberKD); 

    configDefaults();

    m_climberMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //m_climberMotor.set(.1);
    SmartDashboard.putNumber("Climber Pos", m_climberMotor.getPosition().getValueAsDouble());

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

    m_climberMotor.getConfigurator().apply(m_climberMotorConfig);
  }

  private void climberUp() {
    m_climberMotor.set(m_climberController.calculate(m_climberMotor.getPosition().getValueAsDouble(), 
                                                     ClimberConstants.maxHeightPos));                          
  }

  private void climberDown() {
    m_climberMotor.set(m_climberController.calculate(m_climberMotor.getPosition().getValueAsDouble(), 
                                                     ClimberConstants.minHeightPos));                          
  }
}
