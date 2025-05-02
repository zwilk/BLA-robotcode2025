// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int kFrontLeftMotor = 1;
        public static final int kRearLeftMotor = 2;
        public static final int kFrontRightMotor = 3;
        public static final int kRearRightMotor = 4;

        public static final double kDefaultSpeed = 1.0;
        public static final double kAutonSpeed = 0.25;
        public static final double kSlowDriveSpeed = 0.5;
        public static final double kTurningSlowDown = 0.75;

        public static final double kS = 0.65544;
        public static final double kV = 2.3315;
        public static final double kP = -0.03;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final int kPigeonID = 12;

        public static final double kWheelDiameterMeters = Units.inchesToMeters(6);
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double kEncoderCPR = 2048;
        public static final double kGearRatio = 10.71;
        public static final double kMetersPerTick = kEncoderCPR * kGearRatio * kWheelCircumferenceMeters;
    }

    public static final class RollerConstants {
        public static final int ROLLER_MOTOR_ID = 6;
        public static final int ROLLER_MOTOR_CURRENT_LIMIT = 80;
        public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
        public static final double ROLLER_EJECT_VALUE = 0.7;
        public static final double ROLLER_INTAKE_VALUE = 0.7;
      }
    
      public static final class ArmConstants {
        public static final int ARM_LEADER_ID = 5;
        public static final int ARM_MOTOR_CURRENT_LIMIT = 80;
        public static final double ARM_MOTOR_VOLTAGE_COMP = 10;
        public static final double ARM_UP_VALUE = 0.5;
        public static final double ARM_DOWN_VALUE = 0.3;
      } 
      
      public static final class ClimbConstants {
        public static final int CLIMB_MOTOR_ID = 7;
        public static final int CLIMB_MOTOR_CURRENT_LIMIT = 80;
        public static final double CLIMB_MOTOR_VOLTAGE_COMP = 10;
        public static final double CLIMB_VALUE = 0.7;
      }
    
      public static final class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    
      }
    }