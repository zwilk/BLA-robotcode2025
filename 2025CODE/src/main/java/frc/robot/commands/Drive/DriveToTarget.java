// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class DriveToTarget extends PIDCommand {
  private final Drivetrain m_drive;

  /** Creates a new TurnToTarget. */
  public DriveToTarget(Drivetrain drive, Limelight light) {
    super(
        // The controller that the command will use
        new PIDController(-0.2, 0, 0.05),
        // This should return the measurement
        () -> light.getDistance(),
        // This should return the setpoint (can also be a constant)
        () -> 2.12,
        // This uses the output
        output -> {
          // Use the output here
          drive.driveCartesian(output, 0, 0);
        });
        m_drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
