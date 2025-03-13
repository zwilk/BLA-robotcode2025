// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class TurnToTargetPID extends Command {
  private final DoubleSupplier m_ySpeed;
  private final DoubleSupplier m_xSpeed;
  private final Drivetrain m_drive;
  private final Limelight m_light;

  /** Creates a new TurnToTargetPID. */
  public TurnToTargetPID(DoubleSupplier ySpeed, DoubleSupplier xSpeed, Drivetrain drive, Limelight light) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ySpeed = ySpeed;
    m_xSpeed = xSpeed;
    m_drive = drive;
    m_light = light;
    // addRequirements(m_drive, m_light);
  }

  /** Creates a new TurnToTargetPID. */
  public TurnToTargetPID(Drivetrain drive, Limelight light) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ySpeed = () -> 0;
    m_xSpeed = () -> 0;
    m_drive = drive;
    m_light = light;
    // addRequirements(m_drive, m_light);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.getPIDController().setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.driveCartesian(m_ySpeed.getAsDouble(), m_xSpeed.getAsDouble(), m_drive.getPIDController().calculate(m_light.getTX()));
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
