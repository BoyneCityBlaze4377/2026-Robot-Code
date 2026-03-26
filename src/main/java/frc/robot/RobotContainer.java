package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IOConstants;
import frc.robot.Subsystems.*;

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
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("shoot", m_shooter.runIndex());
    //Configure the trigger bindings
    configureBindings();
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
    new JoystickButton(m_tempStick, 6).whileTrue(m_shooter.runIndex());
    new JoystickButton(m_tempStick, 1).whileTrue(m_collector.Collect()).onFalse(m_collector.retractCollector());
    // new JoystickButton(m_driverStick, IOConstants.quickBrakeButtonID).toggleOnTrue(m_driveTrain.QuickBrake());
    // new JoystickButton(m_driverStick, IOConstants.slowModeButtonID).whileTrue(m_driveTrain.);
  //   new JoystickButton(m_driverStick, IOConstants.lockPoseButtonID).whileTrue(LockPose);
  //   new JoystickButton(m_driverStick, IOConstants.switchOrientationButtonID).onTrue(SwitchOrientation);
    // new JoystickButton(m_driverStick, IOConstants.robotOrientButtonID).whileTrue(m_driveTrain.RobotOriented());

    new JoystickButton(m_tempStick, 2).whileTrue(m_climber.ClimberUp());
    new JoystickButton(m_tempStick, 3).whileTrue(m_climber.ClimberDown());
    new JoystickButton(m_tempStick, 4).whileTrue(m_climber.TEMPCLIMBDOWNCMD());
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return null;
    return new PathPlannerAuto("TEST");
  }
}
