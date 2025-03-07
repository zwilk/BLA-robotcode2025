// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Auton.OneBall;
import frc.robot.commands.Auton.OneBallWithLight;
import frc.robot.commands.Auton.ThreeBallRightSide;
import frc.robot.commands.Auton.TwoBallLeftSide;
import frc.robot.commands.Auton.TwoBallLeftSideWithLight;
import frc.robot.commands.Auton.TwoBallRightSide;
import frc.robot.commands.Auton.TwoBallRightSideWithLight;
import frc.robot.commands.Climb.ForceArmsDown;
import frc.robot.commands.Climb.ForceHooksBack;
import frc.robot.commands.Climb.LeanBack;
import frc.robot.commands.Climb.LeanForward;
import frc.robot.commands.Climb.LowerArms;
import frc.robot.commands.Climb.RaiseArms;
import frc.robot.commands.Climb.ResetArmEncoders;
import frc.robot.commands.Climb.ResetHookEncoders;
import frc.robot.commands.Combo.FrontNFeed;
import frc.robot.commands.Combo.SideNFeed;
import frc.robot.commands.Combo.TurnAndShoot;
import frc.robot.commands.Drive.CartesianDrive;
import frc.robot.commands.Drive.DriveToTarget;
import frc.robot.commands.Drive.SlowDrive;
import frc.robot.commands.Drive.ToggleFieldDrive;
import frc.robot.commands.Feeder.FeedBallsDown;
import frc.robot.commands.Feeder.FeedBallsUp;
import frc.robot.commands.Limelight.LEDOff;
import frc.robot.commands.Limelight.LEDOn;
import frc.robot.commands.Shoot.BackwardsShooter;
import frc.robot.commands.Shoot.FastShoot;
import frc.robot.commands.Shoot.LowerHubShoot;
import frc.robot.commands.Shoot.MidShoot;
import frc.robot.commands.Shoot.SlowShoot;
import frc.robot.commands.Shoot.StopShooter;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Drivetrain m_drive = new Drivetrain();
  public final Intake m_intake = new Intake();
  public final Feeder m_feeder = new Feeder();
  public final Shooter m_shooter = new Shooter();
  public final Climb m_climb = new Climb();
  public final Limelight m_light = new Limelight();

  // Controllers
  // public final XboxController m_xboxDriver = new XboxController(OIConstants.kXboxDriverController);
  public final FlightStick m_flightDriver = new FlightStick(OIConstants.kFlightDriverController);
  public final Joystick m_operator = new Joystick(OIConstants.kOperatorController);

  public final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
  public final SendableChooser<Command> m_auto = new SendableChooser<>();

  // Cameras
  public final UsbCamera m_frontCamera;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_frontCamera = CameraServer.startAutomaticCapture();

    // m_drive.setDefaultCommand(new CartesianDrive(
    //   () -> -m_xboxDriver.getLeftY() + -m_flightDriver.getY(),
    //   () -> m_xboxDriver.getLeftX() + m_flightDriver.getX(),
    //   () -> m_xboxDriver.getRightX() + m_flightDriver.getZ(),
    //   m_drive)
    // );

    m_drive.setDefaultCommand(new CartesianDrive(
      () -> -m_flightDriver.getY(),
      () -> m_flightDriver.getX(),
      () -> m_flightDriver.getZ(),
      m_drive)
    );

    m_tab.add("Auton List", m_auto).withPosition(0, 2).withSize(2, 1).withWidget(BuiltInWidgets.kComboBoxChooser);
    m_auto.addOption("One Ball with Light", new OneBallWithLight(m_shooter, m_drive, m_feeder, m_light));
    m_auto.addOption("Two Ball Left Side With Light", new TwoBallLeftSideWithLight(m_drive, m_intake, m_feeder, m_shooter, m_light));
    m_auto.addOption("Two Ball Right Side with Light", new TwoBallRightSideWithLight(m_drive, m_intake, m_feeder, m_shooter, m_light));
    m_auto.addOption("Three Ball Right Side with Light", new ThreeBallRightSide(m_drive, m_intake, m_feeder, m_shooter, m_light));
    m_auto.addOption("One Ball", new OneBall(m_shooter, m_drive, m_feeder));
    m_auto.setDefaultOption("Two Ball Left Side", new TwoBallLeftSide(m_drive, m_intake, m_feeder, m_shooter));
    m_auto.addOption("Two Ball Right Side", new TwoBallRightSide(m_drive, m_intake, m_feeder, m_shooter));

    m_tab.add(m_frontCamera).withPosition(6, 0).withSize(4, 4).withWidget(BuiltInWidgets.kCameraStream);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(m_xboxDriver, Button.kA.value).whenPressed(new ToggleFieldDrive(m_drive));
    // new JoystickButton(m_xboxDriver, Button.kB.value).whenHeld(new SlowDrive(m_drive));
    // new JoystickButton(m_xboxDriver, Button.kLeftBumper.value).whenHeld(new FrontNFeed(m_intake, m_feeder));
    // new JoystickButton(m_xboxDriver, Button.kRightBumper.value).whenHeld(new SideNFeed(m_intake, m_feeder));


    new JoystickButton(m_flightDriver, FlightStick.Button.kTrigger.value).whenHeld(new SlowDrive(m_drive));
    new JoystickButton(m_flightDriver, FlightStick.Button.kLeftTopMiddle.value).whenHeld(new FrontNFeed(m_intake, m_feeder));
    new JoystickButton(m_flightDriver, FlightStick.Button.kLeftTopRight.value).whenHeld(new SideNFeed(m_intake, m_feeder));
    new JoystickButton(m_flightDriver, FlightStick.Button.kLeftTopLeft.value).whenPressed(new ToggleFieldDrive(m_drive));
    // new JoystickButton(m_flightDriver, FlightStick.Button.kMiddleThumb.value).whenHeld(new TurnToTargetPID(() -> -m_flightDriver.getY(), () -> m_flightDriver.getX(), m_drive, m_light));
    new JoystickButton(m_flightDriver, FlightStick.Button.kMiddleThumb.value).whenHeld(new TurnAndShoot(m_drive, m_light, m_feeder, m_shooter));
    new JoystickButton(m_flightDriver, FlightStick.Button.kRightThumb.value).whenHeld(new DriveToTarget(m_drive, m_light));

    new JoystickButton(m_operator, 1).whenHeld(new FrontNFeed(m_intake, m_feeder));
    new JoystickButton(m_operator, 2).whenHeld(new SideNFeed(m_intake, m_feeder));
    new JoystickButton(m_operator, 3).whenHeld(new FeedBallsUp(m_feeder));
    new JoystickButton(m_operator, 4).whenHeld(new FeedBallsDown(m_feeder));
    new JoystickButton(m_operator, 5).whenHeld(new BackwardsShooter(m_shooter));
    new JoystickButton(m_operator, 6).whenHeld(new SlowShoot(m_shooter));
    new JoystickButton(m_operator, 7).whenHeld(new MidShoot(m_shooter));
    new JoystickButton(m_operator, 8).whenHeld(new FastShoot(m_shooter));
    new JoystickButton(m_operator, 9).whenHeld(new LowerHubShoot(m_shooter));
    // new JoystickButton(m_operator, 10).whenHeld(new AutoRPM(m_shooter, m_light));
    new JoystickButton(m_operator, 11).whenHeld(new RaiseArms(m_climb));
    new JoystickButton(m_operator, 12).whileHeld(new LowerArms(m_climb));
    new JoystickButton(m_operator, 13).whenHeld(new LeanBack(m_climb));
    new JoystickButton(m_operator, 15).whenHeld(new LeanForward(m_climb));
    new JoystickButton(m_operator, 17).whenHeld(new ForceHooksBack(m_climb));
    new JoystickButton(m_operator, 18).whenHeld(new ForceArmsDown(m_climb));
    new JoystickButton(m_operator, 19).whenPressed(new ResetHookEncoders(m_climb));
    new JoystickButton(m_operator, 20).whenPressed(new ResetArmEncoders(m_climb));
    new JoystickButton(m_operator, 21).whenPressed(new StopShooter(m_shooter));
    new JoystickButton(m_operator, 22).whenPressed(new LEDOn(m_light));
    new JoystickButton(m_operator, 23).whenPressed(new LEDOff(m_light));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_auto.getSelected();
  }
}