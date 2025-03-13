// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class TurnToTargetBad extends Command {
  private final Drivetrain m_drive;
  private final Limelight m_light;
  
  /**
   * Command to turn the robot to the hab.
   * @param drive The drivetrain subsystem
   * @param light The limelight subsystem
   */
  public TurnToTargetBad(Drivetrain drive, Limelight light) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_light = light;
    addRequirements(m_drive, m_light);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_light.setLEDMode(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_drive.driveCartesian(0, 0, m_drive.getPIDController().calculate(m_light.getTX()) + m_drive.getFeedforward().calculate(m_light.getTX()));
    if (m_light.getTX() < 5 && m_light.getTX() > -5) {
      m_drive.stopDrive();
    } else if (m_light.getTX() < -5 ) {
      m_drive.driveCartesian(0, 0, -0.25);
    } else if (m_light.getTX() > 5) {
      m_drive.driveCartesian(0, 0, 0.25);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopDrive();
    // m_light.setLEDMode(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return !m_light.getTV();
    return false;
  }
}
