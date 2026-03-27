package frc.robot.Climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Lib.AdvancedPose2D;
import frc.Lib.BlazeMath;
import frc.Lib.Vector3D;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Shooter.ShooterMath;

public class Climber extends SubsystemBase {
  public enum ClimberMode {UP, DOWN, CLIMB};

  private final TalonFX m_climberMotor;
  private final TalonFXConfiguration m_climberMotorConfig;

  private double setpoint = ClimberConstants.minHeightPos;

  private final PIDController m_climberController;

  private double climberSpeed = 0;

  public ClimberMode mode = ClimberMode.DOWN;

  public ClimberState m_desiredState = ClimberState.retracted;
  
  /** Creates a new Climber. */
  public Climber() {
    m_climberMotor = new TalonFX(ClimberConstants.climberMotorID);
    m_climberMotorConfig = new TalonFXConfiguration();

    m_climberController = new PIDController(ClimberConstants.climberKP, 
                                            ClimberConstants.climberKI, 
                                            ClimberConstants.climberKD); 

    configDefaults();

    m_climberMotor.setPosition(ClimberConstants.minHeightPos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (mode) {
      case UP:
        setDesiredState(ClimberState.extended);
        break;
      case DOWN:
        setDesiredState(ClimberState.retracted);
        break;
      case CLIMB:
        setDesiredState(ClimberState.climbed);
        break;
      default:
        setDesiredState(ClimberState.retracted);
        break;
    }

    setpoint = m_desiredState.position;
    moveClimber();

    SmartDashboard.putString("ClimberMode", getCurrentMode().toString());
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

  private void moveClimber() {
    m_climberMotor.set(m_climberController.calculate(getPos(), setpoint));
  }

  private void stopClimber() {
    climberSpeed = 0;
  }

  public void setClimberSetpoint(double target) {
    setpoint = target;
  }

  public void setDesiredState(ClimberState desiredState) {
    m_desiredState = desiredState;
  }

  public double getPos() {
    return m_climberMotor.getPosition().getValueAsDouble();
  }

  public void setMode(ClimberMode newMode) {
    mode = newMode;
  }

  private void TEMPCLIMBDOWN() {
    climberSpeed = .1;
  }

  public ClimberMode getCurrentMode() {
    return mode;
  }

  public Command SetClimbMode(ClimberMode mode) {
    return Commands.runOnce(() -> this.setMode(mode), this);
  }

  public Command TEMPCLIMBDOWNCMD() {
    return Commands.runEnd(() -> this.TEMPCLIMBDOWN(), () -> this.stopClimber(), this);
  }
}
