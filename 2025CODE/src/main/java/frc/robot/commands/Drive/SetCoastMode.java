// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class SetCoastMode extends InstantCommand {
  private final Drivetrain m_drive;

  /**
   * Command to set the drive motors to coast mode.
   * @param drive The drivetrain subsystem
   */
  public SetCoastMode(Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_drive.setNeutralMode(NeutralMode.Coast);
  }
}
