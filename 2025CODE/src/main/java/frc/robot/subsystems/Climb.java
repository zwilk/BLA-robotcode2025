// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMax.IdleMode;
import com.revrobotics.SparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  private final SparkMax m_arm = new SparkMax(ClimbConstants.kArmMotor, MotorType.kBrushless);
  private final SparkMax m_hooks = new SparkMax(ClimbConstants.kHookMotor, MotorType.kBrushless);

  private final RelativeEncoder m_armEncoder = m_arm.getEncoder();
  private final RelativeEncoder m_hookEncoder = m_hooks.getEncoder();

  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
  private final NetworkTableEntry m_armLocation;
  private final NetworkTableEntry m_hookLocation;
  private final NetworkTableEntry m_armCurrent;
  private final NetworkTableEntry m_hookCurrent;

  double highestArmCurrent;
  double highestHookCurrent;

  /**
   * Climb Subsystem. This is the subsystem that controls the climbing mechanism.
   */
  public Climb() {
    m_arm.setInverted(true);
    m_hooks.setInverted(true);

    m_arm.setIdleMode(IdleMode.kBrake);
    m_hooks.setIdleMode(IdleMode.kBrake);

    m_armLocation = m_tab.add("Arm Location", getArmPosition()).withPosition(0, 3).getEntry();
    m_hookLocation = m_tab.add("Hook Location", getHookPosition()).withPosition(2, 3).getEntry();
    m_armCurrent = m_tab.add("Arm Current", getArmCurrent()).withPosition(1, 3).getEntry();
    m_hookCurrent = m_tab.add("Hook Current", getHookCurrent()).withPosition(3, 3).getEntry();
  }

  /**
   * Resets the Hook Encoders to zero.
   */
  public void resetHookEncoder(){
    m_hookEncoder.setPosition(0);
  }

  /**
   * Resets the Arm Encoders to zero.
   */
  public void resetArmEncoder() {
    m_armEncoder.setPosition(0);
  }

  /**
   * Raises the climbing arms.
   */
  public void raiseArms() {
    m_arm.set(ClimbConstants.kArmMotorSpeed);
  }


  /**
   * Lowers the climbing arms.
   */
  public void lowerArms() {
    m_arm.set(-ClimbConstants.kArmMotorSpeed);
  }

  /**
   * Slides the hooks back.
   */
  public void slideHooksBack() {
    m_hooks.set(ClimbConstants.kHookMotorSpeed);
  }

  /**
   * Slides the hooks forward.
   */
  public void slideHooksForward() {
    m_hooks.set(-ClimbConstants.kHookMotorSpeed);
  }

  /**
   * Returns the current position of the hooks.
   */
  public double getHookPosition() {
    return -m_hookEncoder.getPosition();
  }

  /**
   * Returns the current position of the arms.
   */
  public double getArmPosition() {
    return m_armEncoder.getPosition();
  }

  /**
   * Returns the amount of current that the Hook motor is using.
   * @return the amount of current in amps.
   */
  public double getHookCurrent() {
    return m_hooks.getOutputCurrent();
  }

  /**
   * Returns the amount of current that the Arm motor is using.
   * @return the amount of current in amps.
   */
  public double getArmCurrent() {
    return m_arm.getOutputCurrent();
  }

  /**
   * Stops the climbing motor.
   */
  public void stopArms() {
    m_arm.stopMotor();
  }

  /**
   * Stops the hooks motor.
   */
  public void stopHooks() {
    m_hooks.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_armLocation.setNumber(getArmPosition());
    m_hookLocation.setNumber(getHookPosition());
    double armCurrent = getArmCurrent();
    double hookCurrent = getHookCurrent();
    if(armCurrent > highestArmCurrent) {
      highestArmCurrent = armCurrent;
    }
    if (hookCurrent > highestHookCurrent) {
      highestHookCurrent = hookCurrent;
    }
    m_armCurrent.setNumber(highestArmCurrent);
    m_hookCurrent.setNumber(highestHookCurrent);
  }
}
