package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

// Command to run the roller with joystick inputs
public class ArmCommand extends Command {
  private final DoubleSupplier forward;
  private final DoubleSupplier reverse;
  // private final CANRollerSubsystem rollerSubsystem;
  private final ArmSubsystem armSubsystem;

  public ArmCommand(
      DoubleSupplier forward, DoubleSupplier reverse, ArmSubsystem armSubsystem) {
    this.forward = reverse;
    this.reverse = forward;
    this.armSubsystem = armSubsystem;

    addRequirements(this.armSubsystem);
  }

  @Override
  public void initialize() {
  }

  // Runs every cycle while the command is scheduled (~50 times per second)
  @Override
  public void execute() {
    // Run the roller motor at the desired speed
    armSubsystem.runArm(forward.getAsDouble(), reverse.getAsDouble());
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