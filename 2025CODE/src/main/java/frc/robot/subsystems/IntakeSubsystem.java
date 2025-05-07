// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax intakeMotor;
  private final SparkMaxConfig intakeConfig;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    
    intakeMotor = new SparkMax(5, MotorType.kBrushless);

    intakeConfig = new SparkMaxConfig();
    intakeConfig.idleMode(IdleMode.kBrake);
    intakeConfig.inverted(false);
    
    intakeConfig.smartCurrentLimit(IntakeConstants.INTAKE_MOTOR_CURRENT_LIMIT);
    intakeConfig.voltageCompensation(IntakeConstants.INTAKE_MOTOR_VOLTAGE_COMP);

    intakeConfig.apply(intakeConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runMotor(double speed) {
    intakeMotor.set(speed);
  }
}
