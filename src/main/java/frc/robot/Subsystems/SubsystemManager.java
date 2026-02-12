package frc.robot.Subsystems;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemManager extends SubsystemBase {
  private final DriveTrain m_driveTrain;
  private final Shooter m_shooter;
  private final Collector m_collector;
  private final Climber m_climber;
  private final AutoFactory m_autoFactory;

  /** Creates a new SubsystemManager. */
  public SubsystemManager(DriveTrain driveTrain, 
    Shooter shooter, Collector collector, Climber climber, AutoFactory autoFactory) {
    m_driveTrain = driveTrain;
    m_shooter = shooter;
    m_collector = collector;
    m_climber = climber;
    m_autoFactory = autoFactory;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
