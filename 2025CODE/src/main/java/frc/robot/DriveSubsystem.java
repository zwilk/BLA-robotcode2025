package main.java.frc.robot;


//import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveSubsystem extends SubsystemBase {
  private final PWMSparkMax m_frontLeft = new PWMSparkMax(Constants.DriveConstants.kFrontLeftMotorPort);
  private final PWMSparkMax m_rearLeft = new PWMSparkMax(Constants.DriveConstants.kRearLeftMotorPort);
  private final PWMSparkMax m_frontRight = new PWMSparkMax(Constants.DriveConstants.kFrontRightMotorPort);
  private final PWMSparkMax m_rearRight = new PWMSparkMax(Constants.DriveConstants.kRearRightMotorPort);

  private final MecanumDrive m_drive =
      new MecanumDrive(m_frontLeft::set, m_rearLeft::set, m_frontRight::set, m_rearRight::set);

  // The front-left-side drive encoder
  private final Encoder m_frontLeftEncoder =
      new Encoder(
          Constants.DriveConstants.kFrontLeftEncoderPorts[0],
          Constants.DriveConstants.kFrontLeftEncoderPorts[1],
          Constants.DriveConstants.kFrontLeftEncoderReversed);

  // The rear-left-side drive encoder
  private final Encoder m_rearLeftEncoder =
      new Encoder(
          Constants.DriveConstants.kRearLeftEncoderPorts[0],
          Constants.DriveConstants.kRearLeftEncoderPorts[1],
          Constants.DriveConstants.kRearLeftEncoderReversed);

  // The front-right--side drive encoder
  private final Encoder m_frontRightEncoder =
      new Encoder(
          Constants.DriveConstants.kFrontRightEncoderPorts[0],
          Constants.DriveConstants.kFrontRightEncoderPorts[1],
          Constants.DriveConstants.kFrontRightEncoderReversed);

  // The rear-right-side drive encoder
  private final Encoder m_rearRightEncoder =
      new Encoder(
          Constants.DriveConstants.kRearRightEncoderPorts[0],
          Constants.DriveConstants.kRearRightEncoderPorts[1],
          Constants.DriveConstants.kRearRightEncoderReversed);

  // The gyro sensor
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  MecanumDriveOdometry m_odometry =
      new MecanumDriveOdometry(
          Constants.DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new MecanumDriveWheelPositions());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    SendableRegistry.addChild(m_drive, m_frontLeft);
    SendableRegistry.addChild(m_drive, m_rearLeft);
    SendableRegistry.addChild(m_drive, m_frontRight);
    SendableRegistry.addChild(m_drive, m_rearRight);

    // Sets the distance per pulse for the encoders
    m_frontLeftEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
    m_rearLeftEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
    m_frontRightEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
    m_rearRightEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how the robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_frontRight.setInverted(true);
    m_rearRight.setInverted(true);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), getCurrentWheelDistances());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), getCurrentWheelDistances(), pose);
  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
   * speeds have no effect on the angular speed.
   *
   * @param xSpeed Speed of the robot in the x direction (forward/backwards).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      m_drive.driveCartesian(xSpeed, ySpeed, rot, m_gyro.getRotation2d());
    } else {
      m_drive.driveCartesian(xSpeed, ySpeed, rot);
    }
  }

  /** Sets the front left drive MotorController to a voltage. */
  public void setDriveMotorControllersVolts(
      double frontLeftVoltage,
      double frontRightVoltage,
      double rearLeftVoltage,
      double rearRightVoltage) {
    m_frontLeft.setVoltage(frontLeftVoltage);
    m_rearLeft.setVoltage(rearLeftVoltage);
    m_frontRight.setVoltage(frontRightVoltage);
    m_rearRight.setVoltage(rearRightVoltage);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeftEncoder.reset();
    m_rearLeftEncoder.reset();
    m_frontRightEncoder.reset();
    m_rearRightEncoder.reset();
  }

  /**
   * Gets the front left drive encoder.
   *
   * @return the front left drive encoder
   */
  public Encoder getFrontLeftEncoder() {
    return m_frontLeftEncoder;
  }

  /**
   * Gets the rear left drive encoder.
   *
   * @return the rear left drive encoder
   */
  public Encoder getRearLeftEncoder() {
    return m_rearLeftEncoder;
  }

  /**
   * Gets the front right drive encoder.
   *
   * @return the front right drive encoder
   */
  public Encoder getFrontRightEncoder() {
    return m_frontRightEncoder;
  }

  /**
   * Gets the rear right drive encoder.
   *
   * @return the rear right encoder
   */
  public Encoder getRearRightEncoder() {
    return m_rearRightEncoder;
  }

  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        m_frontLeftEncoder.getRate(),
        m_rearLeftEncoder.getRate(),
        m_frontRightEncoder.getRate(),
        m_rearRightEncoder.getRate());
  }

  /**
   * Gets the current wheel distance measurements.
   *
   * @return the current wheel distance measurements in a MecanumDriveWheelPositions object.
   */
  public MecanumDriveWheelPositions getCurrentWheelDistances() {
    return new MecanumDriveWheelPositions(
        m_frontLeftEncoder.getDistance(),
        m_rearLeftEncoder.getDistance(),
        m_frontRightEncoder.getDistance(),
        m_rearRightEncoder.getDistance());
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}
