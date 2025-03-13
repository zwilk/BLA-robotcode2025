// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final WPI_VictorSPX m_frontIntakeMotor = new WPI_VictorSPX(IntakeConstants.kFrontIntakeMotor);
  private final WPI_VictorSPX m_sideIntakeMotor = new WPI_VictorSPX(IntakeConstants.kSideIntakeMotor);

  /**
   * Intake Subsystem
   */
  public Intake() {
    m_frontIntakeMotor.setNeutralMode(NeutralMode.Coast);
    m_sideIntakeMotor.setNeutralMode(NeutralMode.Coast);

    m_frontIntakeMotor.setInverted(InvertType.None);
    m_sideIntakeMotor.setInverted(InvertType.None);
  }

  /**
   * Runs the Front Intake at the Intake Motor Speed Constant.
   */
  public void runFrontIntake() {
    m_frontIntakeMotor.set(IntakeConstants.kIntakeMotorSpeed);
  }

  /**
   * Runs the Side Intake at the Intake Motor Speed Constant.
   */
  public void runSideIntake() {
    m_sideIntakeMotor.set(IntakeConstants.kIntakeMotorSpeed);
  }

  /** Stops both the front intake and the side intake motors. */
  public void stopIntakes() {
    m_frontIntakeMotor.stopMotor();
    m_sideIntakeMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
