// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class SlowDrive extends Command {
  private final Drivetrain m_drive;
 
  /**
   * Command that cuts the speed of the robot in halve when running.
   * @param drive The drivetrain subsystem
   */
  public SlowDrive(Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.slowDriveSpeed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.normalDriveSpeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
