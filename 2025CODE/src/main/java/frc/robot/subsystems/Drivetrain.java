package frc.robot.subsystems;

import java.util.Map;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.function.DoubleSupplier;
import java.util.function.IntFunction;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  private final SparkMax m_frontLeft = new SparkMax(4,MotorType.kBrushless);
  private final SparkMax m_rearLeft = new SparkMax(2,MotorType.kBrushless);
  private final SparkMax m_frontRight = new SparkMax(3,MotorType.kBrushless);
  private final SparkMax m_rearRight = new SparkMax(1,MotorType.kBrushless);
  private MecanumDrive m_drive = new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV);
  private final PIDController m_turnController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

  private final SparkMaxConfig frontLeftConfig;
  private final SparkMaxConfig frontRightConfig;
  private final SparkMaxConfig rearRightConfig;
  private final SparkMaxConfig rearLeftConfig;
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
  private final NetworkTableEntry m_toggle;
  private final NetworkTableEntry m_maxSpeed;
  private final NetworkTableEntry m_gyroAngle;

  public boolean toggleFieldDrive = false;
  public double tempSpeed;

  /**
   * Drivetrain Subsystem. This is the subsystem that controls the drivetrain.
   */
  public Drivetrain() {

    frontLeftConfig.inverted(false);
    frontLeftConfig.idleMode(IdleMode.kCoast);
    frontRightConfig.inverted(false);
    frontRightConfig.idleMode(IdleMode.kCoast);
    rearRightConfig.inverted(false);
    rearRightConfig.idleMode(IdleMode.kCoast);
    rearLeftConfig.inverted(false);
    rearRightConfig.idleMode(IdleMode.kCoast);
    m_frontLeft.configure(frontLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
    m_rearLeft.setNeutralMode(NeutralMode.Coast);
    m_frontRight.setNeutralMode(NeutralMode.Coast);
    m_rearRight.setNeutralMode(NeutralMode.Coast);
    m_rearLeft.setNeutralMode(NeutralMode.Coast);

    m_frontLeft.setInverted(toggleFieldDrive);
    m_rearLeft.setInverted(toggleFieldDrive);
    m_frontRight.setInverted(toggleFieldDrive);
    m_rearRight.setInverted(toggleFieldDrive);
    m_drive.setDeadband(0.1);

  
    // m_toggle = m_tab.add("Field Drive", toggleFieldDrive).withPosition(4, 0).getEntry();
    // m_tab.add("Drivetrain", m_drive).withPosition(0, 0).withSize(4, 2);
    // // m_tab.add("Gyro", m_gyro.getAbsoluteCompassHeading()).withPosition(0, 2).withSize(2, 2).withWidget(BuiltInWidgets.kGyro);
    // m_gyroAngle = m_tab.add("Gyro Angle", getGyroAngle()).withPosition(4, 2).getEntry();
    // m_maxSpeed = m_tab.add("Max Speed", 1.0).withPosition(2, 2).withSize(2, 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
    // // m_tab.add("Turning PID", m_turnController).withPosition(7, 0).withSize(1, 2).withWidget(BuiltInWidgets.kPIDController);

    m_maxSpeed.setDouble(DriveConstants.kDefaultSpeed);
  }

  /**
   * Drive method for Mecanum platform.
   * <p>Angles are measured clockwise from the positive X axis. The robot's speed is independent from its angle or rotation rate.
   * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Forward is positive.
   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Right is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
   */
  public void driveCartesian(double ySpeed, double xSpeed, double zRotation) {
    m_drive.driveCartesian(ySpeed, xSpeed, zRotation * DriveConstants.kTurningSlowDown);
  }

  /**
   * Drive method for Mecanum platform.
   * <p>Angles are measured clockwise from the positive X axis. The robot's speed is independent from its angle or rotation rate.
   * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Forward is positive.
   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Right is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
   * @param gyroAngle The current angle reading from the gyro in degrees around the Z axis. Use this to implement field-oriented controls.
   */
  public void driveCartesian(double ySpeed, double xSpeed, double zRotation, Rotation2d gyroAngle) {
    m_drive.driveCartesian(
      ySpeed, 
      xSpeed,
      zRotation * DriveConstants.kTurningSlowDown, 
      gyroAngle);
  }

  /**
   * Sets the mode of operation during neutral throttle output.
   * @param mode The desired mode of operation when the Controller output throttle is neutral (ie brake/coast)
   */


  public void driveLeftSide(double speed) {
    m_frontLeft.setVoltage(speed);
    m_rearLeft.setVoltage(speed);
  }

  public void driveRightSide(double speed) {
    m_frontRight.setVoltage(speed);
    m_rearRight.setVoltage(speed);
  }

  /**
   * Drive method to drive the robot using voltages.
   * <p>Can only move forward and backwards.
   * @param voltage The voltage for the motors.
   */
  public void driveVoltage(double voltage) {
    m_frontLeft.setVoltage(voltage);
    m_rearLeft.setVoltage(voltage);
    m_frontRight.setVoltage(voltage);
    m_rearRight.setVoltage(voltage);
  }

  /**
   * Configure the scaling factor for using drive method.
   * <p>Default value is 1.0.
   * @param maxSpeed Multiplied with the output percentage computed by the drive functions.
   */
  public void setMaxSpeed(double maxSpeed) {
    m_drive.setMaxOutput(maxSpeed);
  }

  /**
   * Gets the average distance of the two right wheels.
   * @return The distance in meters.
   */
  public double getRightDistance() {
    return ((-m_frontRight.getSelectedSensorPosition() / DriveConstants.kEncoderCPR / DriveConstants.kGearRatio) * DriveConstants.kWheelCircumferenceMeters);
  }

  /**
   * Gets the average distance of the two left wheels.
   * @return The distance in meters.
   */
  public double getLeftDistance() {
    return ((-m_frontLeft.getSelectedSensorPosition() / DriveConstants.kEncoderCPR / DriveConstants.kGearRatio) * DriveConstants.kWheelCircumferenceMeters);

  }

  /**
   * Gets the average distance of both sides of the robot.
   * @return The distance in meters.
   */
  public double getAverageDistance() {
    return (getRightDistance() + getLeftDistance()) / 2;
  }
 /** Resets the drive encoders to currently read a position of 0. */
 public void resetEncoders() {
  m_frontLeft.reset();
  m_rearLeft.reset();
  m_frontRight.reset();
  m_rearRight.reset();
}

  /**
   * Returns the current angle value (in degrees, from -180 to 180) reported by the sensor.
   * @return The current angle value in degrees (-180 to 180).
   */
  public double getGyroAngle() {
    return m_gyro.getYaw();
  }

  /**
   * Toggles between field-oriented contols and not.
   * <p>Updates on the ShuffleBoard.
   */
  public void toggleFieldDrive() {
    toggleFieldDrive = !toggleFieldDrive;
    m_toggle.setBoolean(toggleFieldDrive);
  }

  /**
   * Returns the current driving mode.
   * 
   * <p>true = field-oriented controls
   * <p>false = NOT field-oriented controls
   * @return the current driving mode.
   */
  public boolean getFieldDriveMode() {
    return toggleFieldDrive;
  }

  
  /**
   * Resets the gyro.
   */
  public void resetGyro() {
    m_gyro.setYaw(0);
  }

  /**
   * Slows down the robot drive by the Slow Drive Speed Constant.
   * Will return to the original speed before method was called.
   */
  public void slowDriveSpeed() {
    tempSpeed = m_maxSpeed.getDouble(1.0);
    m_maxSpeed.setDouble(tempSpeed * DriveConstants.kSlowDriveSpeed);
  }

  /**
   * Returns the drive speed to the original speed.
   */
  public void normalDriveSpeed() {
    m_maxSpeed.setDouble(tempSpeed);
  }

  /**
   * Stops the drive motors.
   */
  public void stopDrive() {
    m_drive.stopMotor();
  }

  /**
   * Returns the Feedforward.
   */
  public SimpleMotorFeedforward getFeedforward() {
    return m_feedforward;
  }

  /**
   * Returns the PIDController.
   */
  public PIDController getPIDController() {
    return m_turnController;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setMaxSpeed(m_maxSpeed.getDouble(1.0));
    m_gyroAngle.setNumber(getGyroAngle());
  }
}
