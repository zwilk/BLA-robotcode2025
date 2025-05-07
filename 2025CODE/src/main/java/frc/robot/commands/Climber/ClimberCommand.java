// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Climber;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.climberSubsytem;

// public class ClimberCommand extends Command {
//   private final climberSubsytem m_climberSubsystem;
//   private double m_speed;
//   /** Creates a new IntakeCommands. */
//   public ClimberCommand(climberSubsytem m_climber, double speed) {
  
//     m_climberSubsystem = m_climber;
//     m_speed = speed;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(m_climberSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_climberSubsystem.runMotor(m_speed);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }