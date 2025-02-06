// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.cameraserver.CameraServer;





//import java.util.Timer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private CANSparkMax driveright2;
  private CANSparkMax driveright1;
  private CANSparkMax driveleft1;
  private CANSparkMax driveleft2;

  private CANSparkMax armRight;
  private CANSparkMax armLeft;

  private CANSparkMax ShooterLeft;
  private CANSparkMax ShooterRight;
  private CANSparkMax Intake;

Timer timer = new Timer();

/** These following four statements are for naming the drive controllers.
 * 
 */



Joystick Joy = new Joystick(0);


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Stay Put", kDefaultAuto);
    m_chooser.addOption("Drive Out", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    CameraServer.startAutomaticCapture("camera", 0); //Dev ID is what you set the camera too

  driveleft1 = new CANSparkMax(1, MotorType.kBrushed);
  driveleft2 = new CANSparkMax(2, MotorType.kBrushed);
  driveright2 = new CANSparkMax(3, MotorType.kBrushed);
  driveright1 = new CANSparkMax(4, MotorType.kBrushed);

  armLeft = new CANSparkMax(10, MotorType.kBrushless);
  armRight = new CANSparkMax(9,MotorType.kBrushless);

  ShooterLeft = new CANSparkMax(8, MotorType.kBrushless);
  ShooterRight = new CANSparkMax(6, MotorType.kBrushless);

  Intake = new CANSparkMax(7, MotorType.kBrushless);

    driveleft1.setInverted(true);
    driveleft2.setInverted(true);
    driveright1.setInverted(false);
    driveright2.setInverted(false);

    armLeft.setInverted(false);
    armRight.setInverted(true);

    armLeft.setIdleMode(IdleMode.kBrake);
    armRight.setIdleMode(IdleMode.kBrake);
    


    driveleft1.setIdleMode(IdleMode.kBrake);
    driveleft2.setIdleMode(IdleMode.kBrake);
    driveright1.setIdleMode(IdleMode.kBrake);
    driveright2.setIdleMode(IdleMode.kBrake);


    /** These statements ensure that the brake is engaged when joystick is centered/not moved */
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  
    timer.reset();
    timer.start();
  
  }
  
  public void setDriveMotors(double forward, double turn) {
    double left = forward - turn;
    double right = forward + turn;

    driveleft1.set(left);
    driveleft2.set(left);
    driveright1.set(right);
    driveright2.set(right);

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        if(timer.get() < 2.5){setDriveMotors(.4, 0);;}
        else{setDriveMotors(0, 0);}
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    double driveSpeed = -(Joy.getRawAxis(1)*0.75);

if( (Math.abs(Joy.getRawAxis(1)) >0.25) || (Math.abs(Joy.getRawAxis(0)) > 0.25)) {
    setDriveMotors(driveSpeed, (-Joy.getRawAxis(0)*0.7));
} else {
  setDriveMotors(0,0);
}

//This next section is moving the arm up and down using two buttons

if(Joy.getRawButton(5)){
  armRight.set(.2);
  armLeft.set(.2);
 } else if(Joy.getRawButton(3)){
  armRight.set(-0.4);
  armLeft.set(-0.4);
 } else {
  armRight.set(0);
  armLeft.set(0);
 }

 if(Joy.getRawButton(2)){Intake.set(.6);} else {Intake.set(0);} // Side trigger button is intake button, hold to use

 if(Joy.getRawButton(4)){
  ShooterLeft.set(1);
  ShooterRight.set(1);
} else {
  ShooterLeft.set(0);
  ShooterRight.set(0);
}
//Below means top right button of joystick inverts intake
if(Joy.getRawButton(6)){
  Intake.set(-0.6);
}


  }
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

