// // // Copyright (c) FIRST and other WPILib contributors.
// // // Open Source Software; you can modify and/or share it under the terms of
// // // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// // import com.revrobotics.spark.SparkBase.PersistMode;
// // import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.SparkMax;
// // import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// // import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// // import frc.robot.Constants.ArmConstants;
// /*import frc.robot.Constants.ClimbConstants;

// JG 3/26: commented out this import, since I did not 
// understand how to use the constant values
// in the list for the climber

// */

// public class climberSubsytem extends SubsystemBase {
//   /** Creates a new climberSubsytem. */
//   private final SparkMax climbLeader;
//   private final SparkMaxConfig climbConfig;
//   public climberSubsytem() {
// climbLeader = new SparkMax(7,MotorType.kBrushless);

// // Set can timeout. Because this project only sets parameters / once on
//     // construction, the timeout can be long without blocking robot operation. Code
//     // which sets or gets parameters during operation may need a shorter timeout.
//     climbLeader.setCANTimeout(250);
//     climbConfig = new SparkMaxConfig();
//     climbConfig.idleMode(IdleMode.kBrake);
//     climbConfig.inverted(false);

//     climbConfig.apply(climbConfig);
//   }
//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
//   public void runMotor(double speed) {
//     climbLeader.set(speed);
//   }
// }
