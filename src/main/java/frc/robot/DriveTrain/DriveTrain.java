package frc.robot.DriveTrain;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Lib.AdvancedPose2D;
import frc.Lib.ElasticUtil;
import frc.Lib.ElasticUtil.Notification;
import frc.Lib.ElasticUtil.Notification.NotificationLevel;
import frc.Lib.LimelightHelpers;
import frc.Lib.LimelightHelpers.PoseEstimate;
import frc.Lib.TimedValue;
import frc.Lib.Vector3D;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;

public class DriveTrain extends SubsystemBase {
  public enum DriveTrainZoneState {AllianceZone, NeutralZone}
  public enum DriveTrainMode {TELEOP_DEFAULT, TELEOP_SHOOTING, TELEOP_COLLECTING, AUTON_BALLTRACKING};

  private final SwerveModule m_frontLeft, m_frontRight, m_backLeft, m_backRight;
  private final AHRS m_gyro;

  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d estimateField;

  private RobotConfig pathPlannerConfig = AutonConstants.robotConfig;

  private final GenericEntry robotHeading, xSpeedSender, ySpeedSender, omegaSender, matchTime, atDesPose, orientationSender;

  private final PIDController xController = new PIDController(AutoAimConstants.transkP,
                                                              AutoAimConstants.transkI,
                                                              AutoAimConstants.transkD);
  private final PIDController yController = new PIDController(AutoAimConstants.transkP,
                                                              AutoAimConstants.transkI,
                                                              AutoAimConstants.transkD);
  private final PIDController headingController = new PIDController(AutoAimConstants.turnkP,
                                                                    AutoAimConstants.turnkI,
                                                                    AutoAimConstants.turnkD);

  private final String limeLightName = SensorConstants.limeLightName;

  private final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  private final PhotonCamera m_frontCam = new PhotonCamera(SensorConstants.frontCameraName);
  private final PhotonCamera m_sideCam = new PhotonCamera(SensorConstants.frontCameraName);

  private final PhotonPoseEstimator m_frontEstimator = new PhotonPoseEstimator(tagLayout, SensorConstants.frontCamRobotToCam);
  private final PhotonPoseEstimator m_sideEstimator = new PhotonPoseEstimator(tagLayout, SensorConstants.sideCamRobotToCam);

  private Optional<EstimatedRobotPose> frontCamEstPos = Optional.empty(), sideCamEstPos = Optional.empty();

  private TimedValue lastAccel;

  private final Debouncer crashDetectDebouncer = new Debouncer(DriveConstants.crashDebounceTime);

  private DriveTrainZoneState currentZone = DriveTrainZoneState.AllianceZone;
  public DriveTrainMode currentMode = DriveTrainMode.TELEOP_DEFAULT;

  private AdvancedPose2D initialPose = new AdvancedPose2D(13, 1), lastPose;
  private ChassisSpeeds currentSpeeds = new ChassisSpeeds();

  private boolean fieldOrientation = true, isBrake = true, autonInRange = false, notified = false, 
                  crash = false, hasCrashed = false, isCollecting = false, isShooting = false;

  private double tx, ty, ta, tID, heading, x, y, omega;
  private int periodicTimer = 1;

  private final SlewRateLimiter transLimiter = new SlewRateLimiter(DriveConstants.maxAccelerationMetersPerSecondSquared);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.maxRotationAccelerationRadiansPerSecondSquared);
    
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    /* Swerve Modules */
    m_frontLeft = new SwerveModule("frontLeft", ModuleConstants.frontLeftDriveMotorID, 
                                                     ModuleConstants.frontLeftTurningMotorID,
                                                     ModuleConstants.frontLeftTurningEncoderID, 
                                                     ModuleConstants.frontLeftDriveMotorReversed,
                                                     ModuleConstants.frontLeftTurningMotorReversed, 
                                                     ModuleConstants.frontLeftAbsoluteEncoderOffset, 
                                                     ModuleConstants.frontLeftAbsReversed);

    m_frontRight = new SwerveModule("frontRight", ModuleConstants.frontRightDriveMotorID, 
                                                       ModuleConstants.frontRightTurningMotorID,
                                                       ModuleConstants.frontRightTurningEncoderID, 
                                                       ModuleConstants.frontRightDriveMotorReversed,
                                                       ModuleConstants.frontRightTurningMotorReversed, 
                                                       ModuleConstants.frontRightAbsoluteEncoderOffset, 
                                                       ModuleConstants.frontRightAbsReversed);

    m_backLeft = new SwerveModule("backLeft", ModuleConstants.backLeftDriveMotorID, 
                                                   ModuleConstants.backLeftTurningMotorID,
                                                   ModuleConstants.backLeftTurningEncoderID, 
                                                   ModuleConstants.backLeftDriveMotorReversed,
                                                   ModuleConstants.backLeftTurningMotorReversed, 
                                                   ModuleConstants.backLeftAbsoluteEncoderOffset, 
                                                   ModuleConstants.backLeftAbsReversed);

    m_backRight = new SwerveModule("backRight", ModuleConstants.backRightDriveMotorID, 
                                                     ModuleConstants.backRightTurningMotorID,
                                                     ModuleConstants.backRightTurningEncoderID, 
                                                     ModuleConstants.backRightDriveMotorReversed,
                                                     ModuleConstants.backRightTurningMotorReversed, 
                                                     ModuleConstants.backRightAbsoluteEncoderOffset, 
                                                     ModuleConstants.backRightAbsReversed);

    brakeAll();
    resetEncoders();

    // DriveTrain GyroScope
    m_gyro = new AHRS(NavXComType.kUSB1);
    m_gyro.reset();
    m_gyro.reset();
    m_gyro.reset();
    m_gyro.reset();
    m_gyro.reset();
    m_gyro.reset();
    m_gyro.reset();
    m_gyro.reset();
    m_gyro.reset();
    m_gyro.reset();
    m_gyro.reset();
    m_gyro.reset();
    m_gyro.reset();
    m_gyro.reset();
    m_gyro.zeroYaw();
    // m_gyro.setAngleAdjustment(initialPose.getRotation().getDegrees());
    //heading = initialPose.getHeadingDegrees();

    /* Pose Estimation */
    estimateField = new Field2d();
    poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.driveKinematics, 
                                                 getHeading(), 
                                                 getSwerveModulePositions(), 
                                                 initialPose,
                                                 AutoAimConstants.poseEstimateOdometryStdDev,
                                                 AutoAimConstants.poseEstimateVisionStdDev);
    setInitialPose(initialPose);
    lastPose = initialPose;
    estimateField.setRobotPose(initialPose);

    /* DashBoard Initialization */
    robotHeading = IOConstants.TeleopTab.add("Robot Heading", heading)
                                        .withWidget("Gyro")
                                        .withProperties(Map.of("counter_clockwise_positive", true))
                                        .getEntry();
    xSpeedSender = IOConstants.TeleopTab.add("xSpeed", 0)
                                        .withWidget("Number Slider")
                                        .withProperties(Map.of("min_value", -1, "max_value", 1))
                                        .getEntry();
    ySpeedSender = IOConstants.TeleopTab.add("ySpeed", 0)
                                        .withWidget("Number Slider")
                                        .withProperties(Map.of("min_value", -1, "max_value", 1))
                                        .getEntry();
    omegaSender = IOConstants.TeleopTab.add("rot", 0)
                                       .withWidget("Number Slider")
                                       .withProperties(Map.of("min_value", -1, "max_value", 1))
                                       .getEntry();
    matchTime = IOConstants.TeleopTab.add("Match Time", 15)
                                     .withWidget("Match Time")
                                     .withProperties(Map.of("red_start_time", 10, "yellow_start_time", 20))
                                     .getEntry();
    atDesPose = IOConstants.TeleopTab.add("At Desired Pose", false)
                                     .withWidget("Boolean Box")
                                     .getEntry();
    orientationSender = IOConstants.TeleopTab.add("Field Oriented?", true)
                                             .withWidget("Boolean Box")
                                             .getEntry();
    SmartDashboard.putData("Field Position", estimateField);

    //SwerveDrive Widget
    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> m_frontLeft.getState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> m_frontLeft.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Front Right Angle", () -> m_frontRight.getState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> m_frontRight.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Left Angle", () -> m_backLeft.getState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> m_backLeft.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Right Angle", () -> m_backRight.getState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> m_backRight.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Robot Angle", () -> getHeading().getRadians(), null);
      }
    });

    //SwerveDrive DesiredState Widget
    SmartDashboard.putData("Desired Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> m_frontLeft.getDesiredState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> m_frontLeft.getDesiredState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Front Right Angle", () -> m_frontRight.getDesiredState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> m_frontRight.getDesiredState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Left Angle", () -> m_backLeft.getDesiredState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> m_backLeft.getDesiredState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Right Angle", () -> m_backRight.getDesiredState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> m_backRight.getDesiredState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Robot Angle", () -> isCollecting ? headingController.getSetpoint() 
                                                                        : getHeading().getRadians(), null);
      }
    });

    /* PID Controllers */
    xController.setTolerance(AutoAimConstants.transkTolerance);
    yController.setTolerance(AutoAimConstants.transkTolerance);
    headingController.setTolerance(AutoAimConstants.turnkTolerance);
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    /* LimeLight Initialization */
    LimelightHelpers.SetRobotOrientation(limeLightName, initialPose.getRotation().getDegrees(), 
                                         0, 0, 0, 0, 0);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camera_robotspace_set")
                    .setDoubleArray(SensorConstants.limelightRobotSpacePose);

    /* PhotonVision Initialization */
    PhotonPipelineResult FCResult = m_frontCam.getLatestResult();
    frontCamEstPos = m_frontEstimator.estimateCoprocMultiTagPose(FCResult);
    if (frontCamEstPos.isEmpty()) frontCamEstPos = m_frontEstimator.estimateLowestAmbiguityPose(FCResult);

    PhotonPipelineResult SCResult = m_frontCam.getLatestResult();
    sideCamEstPos = m_frontEstimator.estimateCoprocMultiTagPose(SCResult);
    if (sideCamEstPos.isEmpty()) sideCamEstPos = m_frontEstimator.estimateLowestAmbiguityPose(SCResult);

    /* Driving variables initialization */

    fieldOrientation = true;

    x = 0;
    y = 0;
    omega = 0;

    lastAccel = new TimedValue(0, 0);

    headingController.enableContinuousInput(-Math.PI, Math.PI);

    //PathPlanner
    try{
      pathPlannerConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    AutoBuilder.configure(this::getPose,
                          this::setInitialPose,
                          this::getChassisSpeeds, 
                          this::PPDrive,
                          new PPHolonomicDriveController(
                            new PIDConstants(AutonConstants.PPtranskP ,AutonConstants.PPtranskI,AutonConstants.PPtranskD),
                            new PIDConstants(AutonConstants.PPturnkP, AutonConstants.PPturnkI, AutonConstants.PPturnkD)), 
                          pathPlannerConfig,
                          () -> false,
                          this);
  }

  @Override
  public void periodic() {
    if (periodicTimer >= 9) {
      m_frontLeft.update();
      m_frontRight.update();
      m_backLeft.update();
      m_backRight.update();

      periodicTimer = 0;
    }

    //heading = MathUtil.inputModulus(poseEstimator.getEstimatedPosition().getRotation().getDegrees(), -180, 180);
    heading = MathUtil.inputModulus(m_gyro.getAngle(), -180, 180);

    /* Pose Estimation */
    //LimeLight
    Optional<PoseEstimate> LLEstPos = Optional.of(DriverStation.getAlliance().get() == Alliance.Red ? 
      LimelightHelpers.getBotPoseEstimate_wpiRed(limeLightName) : 
      LimelightHelpers.getBotPoseEstimate_wpiBlue(limeLightName));

    //PhotonVision
    PhotonPipelineResult FCResult = m_frontCam.getLatestResult();
    frontCamEstPos = m_frontEstimator.estimateCoprocMultiTagPose(FCResult);
    if (frontCamEstPos.isEmpty()) frontCamEstPos = m_frontEstimator.estimateLowestAmbiguityPose(FCResult);

    PhotonPipelineResult SCResult = m_sideCam.getLatestResult();
    sideCamEstPos = m_sideEstimator.estimateCoprocMultiTagPose(SCResult);
    if (sideCamEstPos.isEmpty()) sideCamEstPos = m_sideEstimator.estimateLowestAmbiguityPose(SCResult);

    //Final Updating
    poseEstimator.update(getHeading(), getSwerveModulePositions());
    if (LLEstPos.get().tagCount >= 1) {
      // Pose2d LLEST = LLEstPos.get().pose;
      // AdvancedPose2D EST = new AdvancedPose2D(LLEST);
      // if (DriverStation.getAlliance().get()==Alliance.Red) {
      //   EST = new AdvancedPose2D(LLEST).flipBoth();
      // }

      poseEstimator.addVisionMeasurement(LLEstPos.get().pose, 
                                         LLEstPos.get().timestampSeconds);
      estimateField.getObject("LLEST").setPose(LLEstPos.get().pose);
    }

    if (!frontCamEstPos.isEmpty()) poseEstimator.addVisionMeasurement(frontCamEstPos.get().estimatedPose.toPose2d(), 
                                                                      frontCamEstPos.get().timestampSeconds);
    if (!sideCamEstPos.isEmpty()) poseEstimator.addVisionMeasurement(sideCamEstPos.get().estimatedPose.toPose2d(), 
                                                                     sideCamEstPos.get().timestampSeconds);

    // Field Displaying
    estimateField.setRobotPose(new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), getHeading()));
    estimateField.getObject("LLEST").setPose(LLEstPos.get().pose);


    /* PPLogging */
    PathPlannerLogging.setLogActivePathCallback((poses) -> estimateField.getObject("Current PP Path").setPoses(poses));

    /** Dashboard Posting */
    robotHeading.setDouble(heading);
    atDesPose.setBoolean(atSetpoints());
    matchTime.setDouble(DriverStation.getMatchTime());
    orientationSender.setBoolean(fieldOrientation);

    // Tell if gyro disconnects
    if (!m_gyro.isConnected() && !notified) {
      ElasticUtil.sendNotification(new Notification(NotificationLevel.ERROR, "NAVX", "GYRO DISCONNECTED")
                                               .withDisplaySeconds(10));
      notified = true;
    } else if (notified && m_gyro.isConnected()) {
      ElasticUtil.sendNotification(new Notification(NotificationLevel.INFO, "NAVX", "GYRO RECONNECTED")
                                               .withDisplaySeconds(5));
      notified = false;
    }

    /* LIMELIGHT */
    NetworkTable table = NetworkTableInstance.getDefault().getTable(SensorConstants.limeLightName);
    tx = table.getEntry("tx").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    ta = table.getEntry("ta").getDouble(0);
    tID = table.getEntry("fID").getDouble(0);

    LimelightHelpers.SetRobotOrientation(limeLightName, getHeading().getDegrees(), 
                                         0, 0, 0, 0, 0);
    
    // Update random stuff
    isBrake = m_frontLeft.getNeutralMode() == NeutralModeValue.Brake;

    if (frontCamEstPos.isPresent()) estimateField.getObject("PHEST").setPose(sideCamEstPos.get().estimatedPose.toPose2d());
    if (sideCamEstPos.isPresent()) estimateField.getObject("SCEST").setPose(sideCamEstPos.get().estimatedPose.toPose2d());


    // Drive Robot
    rawDrive(x , y, omega);

    //Update velocity
    currentSpeeds = SwerveConstants.driveKinematics.toChassisSpeeds(getSwerveModuleStates());
    currentZone = (FieldConstants.allianceZone.pointInZone(new AdvancedPose2D(getPose())) ? 
                   DriveTrainZoneState.AllianceZone : 
                   DriveTrainZoneState.NeutralZone);

    //Crash detection
    // if (crashDetectDebouncer.calculate(Math.abs(getJerk()) > DriveConstants.jerkCrashTheshold)) {
    //   poseEstimator.setVisionMeasurementStdDevs(AutoAimConstants.poseEstimateCrashVisionStdDev);
    //   crash = true;
    //   hasCrashed = true;
    // } else if (crash) {
    //   poseEstimator.setVisionMeasurementStdDevs(AutoAimConstants.poseEstimateVisionStdDev);
    //   crash = false;
    // }

    // lastAccel.setValueAndTime(getAcceleration(), Robot.getRobotTime());
    lastPose = new AdvancedPose2D(getPose());

    if (hasCrashed && lastAccel.getValue() < .01) {
      //setToVisionPos();
      hasCrashed = false;
    }

    if (isShooting) {
      currentMode = DriveTrainMode.TELEOP_SHOOTING;
    } else if (isCollecting) {
      currentMode = DriveTrainMode.TELEOP_COLLECTING;
    } else {
      currentMode = DriveTrainMode.TELEOP_DEFAULT;
    }

    periodicTimer++;
  }

  /**
   * The function that sets the raw speeds of the {@link DriveTrain}
   * 
   * @param xSpeed Speed on the x-axis in Meters per Second
   * @param ySpeed Speed on the y-axis in Meters per Second
   * @param omega Rotational speed in Radians per Second
   */
  private void rawDrive(double xSpeed, double ySpeed, double omega) {
    SmartDashboard.putString("ChassisSpeed Inputs", "X: " + xSpeed + " |Y: " + ySpeed + " |rot: " + omega);

    xSpeedSender.setDouble(xSpeed);
    ySpeedSender.setDouble(ySpeed);
    omegaSender.setDouble(omega);

    ChassisSpeeds desiredSpeeds = fieldOrientation
                           ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omega, getHeading()) 
                           : new ChassisSpeeds(xSpeed, ySpeed, omega);
    SwerveModuleState[] swerveModuleStates = SwerveConstants.driveKinematics.toSwerveModuleStates(desiredSpeeds);

    SmartDashboard.putString("ChassisSpeed Outputs", "X: " + desiredSpeeds.vxMetersPerSecond + 
                                                         " |Y: " + desiredSpeeds.vyMetersPerSecond + 
                                                         " |rot: " + desiredSpeeds.omegaRadiansPerSecond);


    setModuleStates(swerveModuleStates);

    SmartDashboard.putString("DriveTrainMode", currentMode.toString());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   */
  public void teleopDrive(double xSpeed, double ySpeed, double rot) {
    fieldOrientation = true;
    //rot = Math.pow(rot, 3);

    double tempX = MathUtil.applyDeadband(xSpeed, DriveConstants.translationalDeadband);
    double tempY = MathUtil.applyDeadband(ySpeed, DriveConstants.translationalDeadband);
    double tempOmega = MathUtil.applyDeadband(rot, DriveConstants.rotationalDeadband);

    x = tempX * DriveConstants.maxSpeedMetersPerSecond * pickSpeedScaler();
    y = tempY * DriveConstants.maxSpeedMetersPerSecond * pickSpeedScaler();
    omega = //(isCollecting ?  
                // (Math.abs(y) < 1e-6 && Math.abs(x) < 1e-6 ? 0 : 
                //       headingController.calculate(getHeading().getRadians(), Math.atan2(y, x))) : 
                tempOmega * DriveConstants.maxRotationSpeedRadiansPerSecond//)
          * pickSpeedScaler();

    // x = transLimiter.calculate(x);
    // y = transLimiter.calculate(y);
    // omega = rotLimiter.calculate(omega);
  }

  /**
   * Drive the robot accoring to a Choreo Trajectory
   * 
   * @param sample The {@link SwerveSample} by which to drive the robot
   */
  public void choreoDrive(SwerveSample sample) {
      // Generate the next speeds for the robot
      ChassisSpeeds speeds = new ChassisSpeeds(
          sample.vx + xController.calculate(getPose().getX(), sample.x),
          sample.vy + yController.calculate(getPose().getY(), sample.y),
          sample.omega + headingController.calculate(getPose().getRotation().getRadians(), sample.heading)
      );
      
      // Apply the generated speeds
      chassisSpeedDrive(speeds);
      setOrientation(true);
  }

  /**
   * Drive based on a ChassisSpeeds object
   * 
   * @param speeds The desired ChassisSpeeds of the {@link DriveTrain}
   */
  public void chassisSpeedDrive(ChassisSpeeds speeds) {
    x = speeds.vxMetersPerSecond;
    y = speeds.vyMetersPerSecond;
    omega = speeds.omegaRadiansPerSecond;
  }

  public void PPDrive(ChassisSpeeds speeds) {
    x = speeds.vxMetersPerSecond;
    y = speeds.vyMetersPerSecond;
    omega = speeds.omegaRadiansPerSecond;
    fieldOrientation = false;
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.maxModuleSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Set the setpoints of all drive PIDControllers
   * 
   * @param xSetpoint The setpoint for the controller of the x-coordinate
   * @param ySetpoint The setpoint for the controller of the y-coordinate
   * @param headingSetpoint The setpoint for the controller of the robot's heading
   */
  public void setPIDSetpoints(double xSetpoint, double ySetpoint, double headingSetpoint) {
    xController.setSetpoint(xSetpoint);
    yController.setSetpoint(ySetpoint);
    headingController.setSetpoint(headingSetpoint);
  }

  /**
   * Set the setpoints of all drive PIDControllers from a desired position on the field
   * 
   * @param pose The desired position
   */
  public void setPoseSetpoints(AdvancedPose2D pose) {
    setPIDSetpoints(pose.getX(), pose.getY(), pose.getRotation().getRadians());
  }

  /** Drive the robot based on PIDController outputs */
  public void PIDDrive() {
    x = MathUtil.clamp(xController.calculate(getPose().getX()), -DriveConstants.maxSpeedMetersPerSecond,
                                                                 DriveConstants.maxSpeedMetersPerSecond);
    y = MathUtil.clamp(yController.calculate(getPose().getY()), -DriveConstants.maxSpeedMetersPerSecond,
                                                                 DriveConstants.maxSpeedMetersPerSecond);
    omega = MathUtil.clamp(headingController.calculate(getHeading().getRadians()), -DriveConstants.maxRotationSpeedRadiansPerSecond,
                                                                                    DriveConstants.maxRotationSpeedRadiansPerSecond);
  }

  /** @return Whether or not the robot is at its desired position based on PIDController setpoints and tolerances */
  public boolean atSetpoints() {
    return xController.atSetpoint() && yController.atSetpoint() && headingController.atSetpoint();
  }

  /** @return An array of the modules' positions */
  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(),
                                       m_backLeft.getPosition(), m_backRight.getPosition()};
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {m_frontLeft.getState(), m_frontRight.getState(),
                                    m_backLeft.getState(), m_backRight.getState()};
  }

  /** Sets the pose of the robot to be locked: all modules' angles form an X */
  public void lockPose() {
    m_frontLeft.setLockedState(new SwerveModuleState(0, Rotation2d.fromDegrees(225)));
    m_frontRight.setLockedState(new SwerveModuleState(0, Rotation2d.fromDegrees(315)));
    m_backLeft.setLockedState(new SwerveModuleState(0, Rotation2d.fromDegrees(315)));
    m_backRight.setLockedState(new SwerveModuleState(0, Rotation2d.fromDegrees(225)));
  }

  /** @return The current ChassisSpeeds of the {@link DriveTrain} */
  public ChassisSpeeds getChassisSpeeds() {
    return currentSpeeds;
  }

  /**
   * Set the angle offset of the drive gyroscope
   * 
   * @param offsetDeg The desired offset for the gyro
   */
  public void setOffset(double offsetDeg) {
    m_gyro.setAngleAdjustment(offsetDeg);
  }

  /** @return The pose */
  public synchronized Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public AdvancedPose2D getAdvancedPose() {
    return new AdvancedPose2D(getPose());
  }

  public DriveTrainZoneState getCurrentZone() {
    return currentZone;
  }

  public AdvancedPose2D getVelocityVector() {
    return new AdvancedPose2D(getPose()).getDerivativeWith(lastPose);
  }

   /** Switches the idle modes of all modlues' drive motors */
   public void switchBrake() {
    if (isBrake) {
      coastAll();
    } else {
      brakeAll();
    }
  }

  /** Sets all idle modes to Brake */
  public void brakeAll() {
    m_frontLeft.brake();
    m_frontRight.brake();
    m_backLeft.brake();
    m_backRight.brake();
  }

  /** Sets all idle modes to Coast */
  public void coastAll() {
    m_frontLeft.coast();
    m_frontRight.coast();
    m_backLeft.coast();
    m_backRight.coast();
  }

  /** @return The current IdleMode of the {@link DriveTrain} */
  public synchronized NeutralModeValue getNeutralMode() {
    return m_frontLeft.getNeutralMode();
  }

  /**
   * Sets the IdleMode of the {@link DriveTrain}
   * 
   * @param mode The desired IdleMode for the {@link DriveTrain}
   */
  public synchronized void setNeutralMode(NeutralModeValue mode) {
    m_frontLeft.setNeutralMode(mode);
    m_frontRight.setNeutralMode(mode);
    m_backLeft.setNeutralMode(mode);
    m_backRight.setNeutralMode(mode);
  }

  public AdvancedPose2D getTargetFuelMeanPos() {
    PhotonPipelineResult result = m_frontCam.getLatestResult();
    List<PhotonTrackedTarget> targets;
    AdvancedPose2D targetMeanPos = new AdvancedPose2D();

    if (result.hasTargets()) {
      targets = result.getTargets();
      double totalX = 0;
      double totalY = 0;


      for (PhotonTrackedTarget t : targets) {
        totalX += t.getBestCameraToTarget().getX();
        totalY += t.getBestCameraToTarget().getY();
      }
      // Potentially change to median rather than mean
      targetMeanPos = new AdvancedPose2D(totalX / targets.size(), totalY / targets.size());
    }

    return targetMeanPos;
  }

  public void driveToTargetFuel() {
    AdvancedPose2D targetPos = getTargetFuelMeanPos().plus(getAdvancedPose());

    omega = headingController.calculate(getHeading().getDegrees(), targetPos.getRotation().getDegrees());
  }

  /** Stops drive motors for all modules */
  public void stop() {
    x = 0;
    y = 0;
    omega = 0;
  }

  /** Reset encoders of all modules */
  public synchronized void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_backLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backRight.resetEncoders();
  }

  /** Switches whether or not the robot drives field oriented */
  public synchronized void switchOrientation() {
    if (fieldOrientation) {
      fieldOrientation = false;
    } else {
      fieldOrientation = true;
    }
  }

  public double pickSpeedScaler() {
    // if (isShooting) {
    //   return DriveConstants.shootingSpeedScaler;
    // } else if (isCollecting) {
    //   return DriveConstants.collectionSpeedScaler;
    // } else {
      return DriveConstants.defaultSpeedScaler;
    // }
  }

  /**
   * Sets whether or not the robot should drive Field Oriented.
   * 
   * @param isFieldOriented True for Field Oriented, false for Robot Oriented.
   */
  public synchronized void setOrientation(boolean isFieldOriented) {
    fieldOrientation = isFieldOriented;
  }

  /** @return Whether or not the {@link DriveTrain} is field oriented */
  public synchronized boolean isFieldOriented() {
    return fieldOrientation;
  }


  /**
   * Sets the offset of the gyro.
   * 
   * @param offsetDegrees The number of degrees to offset by.
   */
  public void setGyroOffset(double offsetDegrees) {
    m_gyro.setAngleAdjustment(offsetDegrees);
  }

  /** Zeroes the heading of the robot */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /** @return The filtered robot heading as a {@link Rotation2d} */
  public synchronized Rotation2d getHeading() {
    return Rotation2d.fromDegrees(heading);
  }

  public Vector3D getAngularVelocityVector() {
    return new Vector3D(0, 0, -currentSpeeds.omegaRadiansPerSecond);
  }

  /** @return The unfiltered heading of the robot */
  public synchronized double getRawHeading() {
    return m_gyro.getAngle();
  }

   /** @return The roll of the gyro */
   public double getRoll(){
    return m_gyro.getRoll();
  }
  
  /** @return The pitch of the gyro */
  public double getPitch(){
    return m_gyro.getPitch();
  }

  public double getAcceleration() {
    return Math.hypot(m_gyro.getWorldLinearAccelX(), m_gyro.getWorldLinearAccelY()) * Math.sqrt(9.80665);
  }

  public double getJerk() {
    return new TimedValue(getAcceleration(), Robot.getRobotTime()).getRateOfChange(lastAccel);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second.
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.gyroReversed ? -1.0 : 1.0);
  }

  /** @return The average distance of all modules */
  public double getAverageDistance(){
    return (Math.abs(m_frontLeft.getDistance()) + Math.abs(m_frontRight.getDistance()) +
            Math.abs(m_backLeft.getDistance()) + Math.abs(m_backRight.getDistance())) / 4;
    
  }

  /** @return Whether or not the robot is close enough to bring up the elevator in auton */
  public boolean getInRange() {
    return autonInRange;
  }

  /** Set whether or not the robot is close enough to bring up the elevator in auton */
  public void setInRange(boolean isInRange) {
    autonInRange = isInRange;
  }

  public double getTX() {
    return tx;
  }

  public double getTY() {
    return ty;
  }

  public double getTA() {
    return ta;
  }

  public double getTargetID() {
    return tID;
  }

  /**
   * Set the initial pose of the {@link DriveTrain}
   * 
   * @param pose The initial pose to be set
   */
  public synchronized void setInitialPose(Pose2d pose) {
    poseEstimator.resetPosition(getHeading(), getSwerveModulePositions(), pose);
  }

  public void resetPose() {
    poseEstimator.resetPose(initialPose);
  }

  //choreo
  public void followTrajectory(SwerveSample sample) {
    Pose2d pose = getPose();

    ChassisSpeeds speeds = new ChassisSpeeds( 
      sample.vx + xController.calculate(pose.getX(), sample.x), 
      sample.vy + yController.calculate(pose.getY(), sample.y),
      sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading));

    chassisSpeedDrive(speeds);

    SmartDashboard.putString("ChassisSpeeds", speeds.toString());
  }

  public void setCurrentMode(DriveTrainMode newMode) {
    currentMode = newMode;
  }

  public void setIsShooting(BooleanSupplier shooting) {
    isShooting = shooting.getAsBoolean();
  }

  public void setIsCollecting(BooleanSupplier collecting) {
    isCollecting = collecting.getAsBoolean();
  }

  /** COMMANDS */
  public Command TeleopDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    return Commands.runEnd(() -> this.teleopDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), omegaSupplier.getAsDouble()),
                           () -> this.stop(),
                           this);
  }

  public Command SetDriveTrainMode(DriveTrainMode mode) {
    return Commands.runOnce(() -> this.setCurrentMode(mode));
  }

  public Command SetDriveTrainMode(Supplier<DriveTrainMode> modeSupplier) {
    return Commands.runOnce(() -> this.setCurrentMode(modeSupplier.get()));
  }

  public Command QuickBrake() {
    return Commands.startRun(() -> this.brakeAll(), 
                             () -> this.teleopDrive(0, 0, 0), 
                             this);
  }

  public Command RobotOriented() {
    return Commands.runOnce(() -> this.setOrientation(false), 
                            this);
  }
}