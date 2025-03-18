package frc.robot.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RollerSubsystem;

public class SimpleCoralAuto extends Command {
    private Drivetrain m_drive;
    private RollerSubsystem m_roller;
    private ArmSubsystem m_arm;
    private Timer timer;
    private double drive_seconds = 3.25;
    private double exjest_seconds = 4.5;

    /**
     * This auto will have the robot drive forwards, stop then drop the coral into L1
     * 
     * There are many ways to write autos, this form will work well for most simple
     * auto routines. For more advanced routines you may want a different structure and 
     * to use more sensors.
     * 
     * Here we use two timer gates, after the robot has finished driving for the first 3.25 
     * seconds, it will exjest the coral for 4.5-3.25 = 1.25 seconds.
     * 
     * 
     * @param drive
     * @param roller
     * @param arm
     */
    public SimpleCoralAuto(Drivetrain drive, Intake roller, ArmSubsystem arm)
    {
        m_drive = drive;
        m_roller = roller;
        m_arm = arm;
        
        timer = new Timer();

        addRequirements(m_drive);
        addRequirements(m_roller);
        addRequirements(m_arm);
    }

    @Override
  public void initialize() {
    // start timer, uses restart to clear the timer as well in case this command has
    // already been run before
    timer.restart();
  }

  // Runs every cycle while the command is scheduled (~50 times per second)
  @Override
  public void execute() {
    /**
     * We always want to hold the arm up duirng the auto to ensure the rollers
     */ 
    m_arm.runArm(ArmConstants.ARM_HOLD_UP);

    /**
     * While this timer is less than drive_seconds, the robot will obey the command inside
     */
    if(timer.get() < drive_seconds)
    {
        m_drive.driveCartesian(drive_seconds, exjest_seconds, drive_seconds)
    /**
     * Once the timer is greater than drive_seconds but less than exjest seconds,
     * the code inside will run, here we stop the drivetrain and exjest the coral.
     */
    else if(timer.get() > drive_seconds && timer.get() < exjest_seconds)
    {
        m_drive.driveCartesian(drive_seconds, exjest_seconds, drive_seconds);
        m_roller.runRoller(RollerConstants.ROLLER_CORAL_OUT);
    }
  }

  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) {
    // stop drive motors
    m_drive.driveCartesian(0.0, 0.0, false);
    m_roller.runRoller(0);
    timer.stop();
  }

  // Runs every cycle while the command is scheduled to check if the command is
  // finished
  @Override
  public boolean isFinished() {
    // check if timer exceeds seconds, when it has this will return true indicating
    // this command is finished
    return timer.get() >= exjest_seconds;
  }
}


public class SimpleCoralAuto extends Command {
  private DriveSubsystem m_drive;
  private RollerSubsystem m_roller;
  private ArmSubsystem m_arm;
  private Timer timer; // [1]
  private double drive_seconds = 3.25; // [2]
  private double exjest_seconds = 4.5;

  /**
   * This auto will have the robot drive forwards, stop then drop the coral into L1
   * 
   * There are many ways to write autos, this form will work well for most simple
   * auto routines. For more advanced routines you may want a different structure and 
   * to use more sensors.
   * 
   * Here we use two timer gates, after the robot has finished driving for the first 3.25 
   * seconds, it will exjest the coral for 4.5-3.25 = 1.25 seconds.
   * 
   * 
   * @param drive
   * @param roller
   * @param arm
   */
  public SimpleCoralAuto(DriveSubsystem drive, RollerSubsystem roller, ArmSubsystem arm)
  {
      m_drive = drive;
      m_roller = roller;
      m_arm = arm;
      
      timer = new Timer(); // [3]

      addRequirements(m_drive);
      addRequirements(m_roller);
      addRequirements(m_arm);
  }

  @Override
  public void initialize() {
    // start timer, uses restart to clear the timer as well in case this command has
    // already been run before
    timer.restart(); // [4]
  }
}


@Override
  public void execute() {
    /**
     * We always want to hold the arm up duirng the auto to ensure the rollers
     */ 
    m_arm.runArm(ArmConstants.ARM_HOLD_UP); // [1]

    /**
     * While this timer is less than drive_seconds, the robot will obey the command inside
     */
    if(timer.get() < drive_seconds) // [2]
    {
      m_drive.driveCartesian(drive_seconds, exjest_seconds, drive_seconds);
    }
    /**
     * Once the timer is greater than drive_seconds but less than exjest seconds,
     * the code inside will run, here we stop the drivetrain and exjest the coral.
     */
    else if(timer.get() > drive_seconds && timer.get() < exjest_seconds) // [3]
    {
      m_drive.driveCartesian(drive_seconds, exjest_seconds, drive_seconds);
      m_roller.runRoller(RollerConstants.ROLLER_CORAL_OUT);
    }
  }

  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) { // [5]
    // stop drive motors
    m_drive.driveCartesian(0.0, 0.0, false);
    m_roller.runRoller(0);
    timer.stop();
  }

  // Runs every cycle while the command is scheduled to check if the command is
  // finished
  @Override
  public boolean isFinished() { // [4]
    // check if timer exceeds seconds, when it has this will return true indicating
    // this command is finished
    return timer.get() >= exjest_seconds;
  }
}