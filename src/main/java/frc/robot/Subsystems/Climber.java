package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final TalonFX m_stage1, m_stage2;

  private final TalonFXConfiguration m_stage1Config, m_stage2Config;
  /** Creates a new Climber. */
  public Climber() {
    m_stage1 = new TalonFX(ClimberConstants.stage1MotorID);
    m_stage2 = new TalonFX(ClimberConstants.stage2MotorID);

    m_stage1Config = new TalonFXConfiguration();
    m_stage2Config = new TalonFXConfiguration();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
