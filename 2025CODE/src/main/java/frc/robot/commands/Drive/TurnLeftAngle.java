// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class TurnLeftAngle extends Command {
  private final double m_angle;
  private final Drivetrain m_drive;

  /**
   * Command to turn the robot left a certain angle.
   * @param drive The drivetrain subsystem
   * @param angle The angle to turn the robot
   */
  public TurnLeftAngle(double angle, Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_angle = angle;
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.driveCartesian(0, 0, -DriveConstants.kAutonSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_drive.getGyroAngle()) >= m_angle;
  }
}
