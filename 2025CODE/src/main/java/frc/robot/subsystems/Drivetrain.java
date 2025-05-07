package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants; 
//set can id
public class Drivetrain extends SubsystemBase {
  private final SparkMax m_frontLeft = new SparkMax(4,MotorType.kBrushless);
  private final SparkMax m_rearLeft = new SparkMax(2,MotorType.kBrushless);
  private final SparkMax m_frontRight = new SparkMax(3,MotorType.kBrushless);
  private final SparkMax m_rearRight = new SparkMax(1,MotorType.kBrushless);

//move the motors to one object
  private MecanumDrive m_drive = new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);

  private final SparkMaxConfig frontLeftConfig = new SparkMaxConfig();
  private final SparkMaxConfig frontRightConfig = new SparkMaxConfig();
  private final SparkMaxConfig rearRightConfig = new SparkMaxConfig();
  private final SparkMaxConfig rearLeftConfig = new SparkMaxConfig();



  /**
   * Drivetrain Subsystem. This is the subsystem that controls the drivetrain.
   */
  public Drivetrain() {
//set the motor controllers to brake mode and invert if needed
    frontLeftConfig.inverted(true);
    frontLeftConfig.idleMode(IdleMode.kBrake);
    frontRightConfig.inverted(false);
    frontRightConfig.idleMode(IdleMode.kBrake);
    rearRightConfig.inverted(false);
    rearRightConfig.idleMode(IdleMode.kBrake);
    rearLeftConfig.inverted(true);
    rearLeftConfig.idleMode(IdleMode.kBrake);
    //configure the motors to reset after matches
    m_frontLeft.configure(frontLeftConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    m_rearLeft.configure(rearLeftConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    m_frontRight.configure(frontRightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    m_rearRight.configure(rearRightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    //add controller deadband
    m_drive.setDeadband(0.1);
    
  }

  @Override
  public void periodic() {

  }
//sinc the joystick nobs to the motors 
  public void driveMecanum(double xVelocity, double yVelocity, double rotVelocity){
    m_drive.driveCartesian(xVelocity, yVelocity, rotVelocity);
  }

  // public void driveTest(double xVelocity) {
  //   m_frontLeft.set(xVelocity);
  //   m_frontRight.set(xVelocity);
  //   m_rearLeft.set(xVelocity);
  //   m_rearLeft.set(xVelocity);
  // }
  //set the seed ot the left and right drive motors
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
   * Gets the average distance of the two right wheels.
   * @return The distance in meters.
   */
  public double getRightDistance() {
    return ((-m_frontRight.getEncoder().getPosition() / DriveConstants.kEncoderCPR / DriveConstants.kGearRatio) * DriveConstants.kWheelCircumferenceMeters);
  }

  /**
   * Gets the average distance of the two left wheels.
   * @return The distance in meters.
   */
  public double getLeftDistance() {
    return ((-m_frontLeft.getEncoder().getPosition() / DriveConstants.kEncoderCPR / DriveConstants.kGearRatio) * DriveConstants.kWheelCircumferenceMeters);

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
  m_frontLeft.getEncoder().setPosition(0);
  m_rearLeft.getEncoder().setPosition(0);
  m_frontRight.getEncoder().setPosition(0);
  m_rearRight.getEncoder().setPosition(0);;
}

  /**
   * Stops the drive motors.
   */
  public void stopDrive() {
    m_drive.stopMotor();
  }

}