package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import java.util.function.DoubleSupplier;

// Command to run the roller with joystick inputs
public class ClimberCommand extends Command {
  private final DoubleSupplier forward;
  private final DoubleSupplier reverse;
  // private final CANRollerSubsystem rollerSubsystem;
  private final ClimberSubsystem climbSubsystem;

  public ClimberCommand(
      DoubleSupplier forward, DoubleSupplier reverse, ClimberSubsystem climbSubsystem) {
    this.forward = reverse;
    this.reverse = forward;
    this.climbSubsystem = climbSubsystem;

    addRequirements(this.climbSubsystem);
  }

  @Override
  public void initialize() {
  }

  // Runs every cycle while the command is scheduled (~50 times per second)
  @Override
  public void execute() {
    // Run the roller motor at the desired speed
    climbSubsystem.runClimb(forward.getAsDouble(), reverse.getAsDouble());
  }

  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) {
  }

  // Runs every cycle while the command is scheduled to check if the command is
  // finished
  @Override
  public boolean isFinished() {
    // Return false to indicate that this command never ends. It can be interrupted
    // by another command needing the same subsystem.
    return false;
  }
}