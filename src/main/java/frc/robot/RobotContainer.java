package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Climber.Climber;
import frc.robot.Climber.Climber.ClimberMode;
import frc.robot.Collector.Collector;
import frc.robot.Collector.Collector.CollectorMode;
import frc.robot.Constants.IOConstants;
import frc.robot.DriveTrain.DriveTrain;
import frc.robot.DriveTrain.DriveTrain.DriveTrainMode;
import frc.robot.Shooter.Shooter;
import frc.robot.Shooter.Shooter.ShooterMode;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Shooter m_shooter = new Shooter(m_driveTrain::getAdvancedPose,
                                                m_driveTrain::getChassisSpeeds,
                                                m_driveTrain::getAngularVelocityVector,
                                                m_driveTrain::getCurrentZone);
  private final Collector m_collector = new Collector();
  private final Climber m_climber = new Climber();

  private final AutoFactory m_autoFactory = new AutoFactory(m_driveTrain::getPose, 
                                                            m_driveTrain::setInitialPose, 
                                                            m_driveTrain::followTrajectory, 
                                                            false, 
                                                            m_driveTrain);

  private final Joystick m_driverStick = new Joystick(IOConstants.driverControllerID);
  private final Joystick m_tempStick = new Joystick(1);

  private final HubStatusMonitor hubStatusMonitor = new HubStatusMonitor(m_tempStick);

  private final Supplier<DriveTrainMode> changeToDefaultOrNot = () -> (m_shooter.getCurrentMode() == ShooterMode.SHOOTING || 
                                                                       m_shooter.getCurrentMode() == ShooterMode.AIMING ? 
                                                                       DriveTrainMode.TELEOP_SHOOTING : 
                                                                       DriveTrainMode.TELEOP_DEFAULT);
  private final Supplier<ShooterMode> changeToShooterIdleOrNot = () -> (m_collector.getCurrentMode() == CollectorMode.COLLECTING
                                                                        ? ShooterMode.AIMING : ShooterMode.IDLE);
  private final Supplier<DriveTrainMode> changeToCollectingOrNot = () -> (m_collector.getCurrentMode() == CollectorMode.COLLECTING
                                                                          ? DriveTrainMode.TELEOP_COLLECTING
                                                                          : DriveTrainMode.TELEOP_DEFAULT);

  // public final Command TeleopDrive = m_driveTrain.TeleopDrive(() -> -m_tempStick.getRawAxis(IOConstants.xAxis), 
  //                                                             () -> -m_tempStick.getRawAxis(IOConstants.yAxis),
  //                                                             () -> -m_tempStick.getRawAxis(IOConstants.omegaAxis));

  // public final Command Collect = m_collector.SetCollectorMode(CollectorMode.COLLECTING)
  //                                                          .alongWith(m_driveTrain.SetDriveTrainMode(
  //                                                                       DriveTrainMode.TELEOP_COLLECTING));
  // public final Command EndCollect = m_collector.SetCollectorMode(CollectorMode.JOSTLING)
  //                                                             .alongWith(m_driveTrain.SetDriveTrainMode(changeToDefaultOrNot));

  // public final Command ClimberUp = m_climber.SetClimbMode(ClimberMode.UP);
  // public final Command ClimberDown = m_climber.SetClimbMode(ClimberMode.DOWN);
  // public final Command Climb = m_climber.SetClimbMode(ClimberMode.CLIMB);

  // public final Command Shoot = m_shooter.SetShooterToShoot();
  // public final Command ShooterIdle = m_shooter.SetShooterToIdle();
  // public final Command ShooterAiming = m_shooter.SetShooterToAiming();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("shoot", m_shooter.runIndex());
    //Configure the trigger bindings
    configureBindings();
    registerPathPlannerNamedCommands();

    m_driveTrain.setDefaultCommand(m_driveTrain.TeleopDrive(() -> -m_tempStick.getRawAxis(IOConstants.xAxis), 
                                                              () -> -m_tempStick.getRawAxis(IOConstants.yAxis),
                                                              () -> -m_tempStick.getRawAxis(IOConstants.omegaAxis)));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // new Trigger(m_tempStick.axisGreaterThan(3, .2, new EventLoop())).whileTrue(
    //   Commands.repeatingSequence(m_shooter.SetShooterMode(ShooterMode.SHOOTING))).onFalse(
    //   m_shooter.SetShooterMode(ShooterMode.IDLE));

    new JoystickButton(m_tempStick, 6).whileTrue(
      Commands.repeatingSequence(m_shooter.SetShooterMode(ShooterMode.SHOOTING))).onFalse(
      m_shooter.SetShooterMode(ShooterMode.IDLE));

    new JoystickButton(m_tempStick, 1).onTrue(
      m_collector.SetCollectorMode(CollectorMode.COLLECTING)).onFalse(
      m_collector.SetCollectorMode(CollectorMode.RETRACTED));

    new JoystickButton(m_tempStick, 4).onTrue(m_climber.SetClimbMode(ClimberMode.UP));
    new JoystickButton(m_tempStick, 2).onTrue(m_climber.SetClimbMode(ClimberMode.DOWN));
    new JoystickButton(m_tempStick, 3).onTrue(m_climber.SetClimbMode(ClimberMode.CLIMB));
  }

  private void registerPathPlannerNamedCommands() {
    NamedCommands.registerCommand("Shoot", Commands.repeatingSequence(m_shooter.SetShooterMode(ShooterMode.SHOOTING)));
    NamedCommands.registerCommand("ShooterToTrenchMode", m_shooter.SetShooterMode(ShooterMode.TRENCH));
    NamedCommands.registerCommand("ShooterReady", m_shooter.SetShooterMode(ShooterMode.AIMING));
    NamedCommands.registerCommand("ShooterIdle", m_shooter.SetShooterMode(ShooterMode.IDLE));

    NamedCommands.registerCommand("Collect", m_collector.SetCollectorMode(CollectorMode.COLLECTING));
    NamedCommands.registerCommand("CollectorRetract", m_collector.SetCollectorMode(CollectorMode.RETRACTED));
    NamedCommands.registerCommand("CollectorJostle", m_collector.SetCollectorMode(CollectorMode.JOSTLING));

    NamedCommands.registerCommand("ClimberUp", m_climber.SetClimbMode(ClimberMode.UP));
    NamedCommands.registerCommand("ClimberDown", m_climber.SetClimbMode(ClimberMode.DOWN));
    NamedCommands.registerCommand("Climb", m_climber.SetClimbMode(ClimberMode.CLIMB));
  }

  public void periodic() {
    m_driveTrain.setIsCollecting(() -> m_collector.getCurrentMode() == CollectorMode.COLLECTING);
    m_driveTrain.setIsShooting(() -> m_shooter.getCurrentMode() == ShooterMode.SHOOTING);

    m_shooter.setIsCollecting(() -> m_collector.getCurrentMode() == CollectorMode.COLLECTING);

    hubStatusMonitor.update();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return null;
<<<<<<< HEAD
    return new PathPlannerAuto("PreBall2Climb");
=======
    return null;
>>>>>>> c9f893ab12b0062302b088cbc4d0e2f80f2039e7
  }
}
