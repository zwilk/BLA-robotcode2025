// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class ToggleFieldDrive extends InstantCommand {
  private final Drivetrain m_drive;

  /**
   * Command that will toggle field-oriented driving or not.
   * @param drive The drivetrain subsystem
   */
  public ToggleFieldDrive(Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.toggleFieldDrive();
  }
}
