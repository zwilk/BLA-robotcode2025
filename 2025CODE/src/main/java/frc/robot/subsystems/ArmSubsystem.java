package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Class to run the intake over CAN */
public class ArmSubsystem extends SubsystemBase {
  private final SparkMax armLeader;

  public ArmSubsystem() {
    // Set up the intake as brushed motors
    armLeader = new SparkMax(6, MotorType.kBrushless);

   
    armLeader.setCANTimeout(250);

    // Create and apply configuration for intake motor. Voltage compensation helps
    // the intake behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the intake stalls.
    SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig.voltageCompensation(ArmConstants.ARM_MOTOR_VOLTAGE_COMP);
    armConfig.smartCurrentLimit(ArmConstants.ARM_MOTOR_CURRENT_LIMIT);
    armConfig.inverted(false);
    armConfig.idleMode(IdleMode.kBrake);

    armLeader.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
  }

  /** This is a method that makes the intake spin */
  public void runArm(double speed) {
    armLeader.set(speed);
  }
} 