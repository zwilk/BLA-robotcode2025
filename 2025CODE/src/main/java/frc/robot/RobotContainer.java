package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.auto.Autos;
import frc.robot.commands.Arm.ArmCommand;
// import frc.robot.commands.Climber.ClimberCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.climberSubsytem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The autonomous chooser
    SendableChooser<Command> m_chooser = new SendableChooser<>(); // [2]

    public final Drivetrain m_drive = new Drivetrain();

    public final IntakeSubsystem m_intake = new IntakeSubsystem();
    ArmSubsystem m_arm = new ArmSubsystem();
    // climberSubsytem m_Climber = new climberSubsytem();
    // Controllers
    // public final XboxController m_xboxDriver = new XboxController(OIConstants.kXboxDriverController);
    public final CommandXboxController controller = new CommandXboxController(OIConstants.kXboxDriverController);
  
    public final SendableChooser<Command> m_auto = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Add auto choices
    m_auto.setDefaultOption("Drive Speed", Autos.driveForward(m_drive, 0.1));


    configureButtonBindings();

  }
    
    // Example to add more autos
    // m_auto.addOption(null, getAutonomousCommand());
 

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    m_drive.setDefaultCommand(
      Commands.run(
        () -> m_drive.driveMecanum(
          controller.getLeftY(),
          -1 * controller.getLeftX(),
          -1 * controller.getRightX()), 
        m_drive));

    controller.leftBumper().whileTrue(new IntakeCommand(m_intake, 0.5));
    controller.rightBumper().whileTrue(new IntakeCommand(m_intake, -0.5));
    controller.rightBumper().or(controller.leftBumper()).whileFalse(new IntakeCommand(m_intake, 0.0));
    
    controller.leftTrigger().whileTrue(new ArmCommand(m_arm, -0.1));
    controller.rightTrigger().whileTrue(new ArmCommand(m_arm, 0.1));
    controller.leftTrigger().or(controller.rightTrigger()).whileFalse(new ArmCommand(m_arm, 0.0));

    // controller.a().whileTrue(new ClimberCommand(m_Climber, 0.3));
    // controller.b().whileTrue(new ClimberCommand(m_Climber, -0.3));
    // controller.a().or(controller.b()).whileFalse(new ClimberCommand(m_Climber, 0.0));

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