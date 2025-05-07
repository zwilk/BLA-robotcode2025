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
        
        public static final double kDefaultSpeed = 1;
        public static final double kAutonSpeed = 0.25;
        public static final double kSlowDriveSpeed = 0.5;
        public static final double kTurningSlowDown = 0.75;

        public static final double kS = 0.65544;
        public static final double kV = 2.3315;
        public static final double kP = 0.03;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final int kPigeonID = 12;

        public static final double kWheelDiameterMeters = Units.inchesToMeters(6);
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double kEncoderCPR = 2048;
        public static final double kGearRatio = 10.71;
        public static final double kMetersPerTick = kEncoderCPR * kGearRatio * kWheelCircumferenceMeters;
    }

  

    public static final class FeederConstants {
        public static final int kBeltFeederMotor = 7;
        public static final int kFrontFeederMotor = 8;
        public static final double kFeederMotorSpeed = 1.0;
    }

    public static final class ShooterConstants {
        public static final int kShooterMotor = 9;
        public static final double kSlowShootRPM = 3800; // 3600
        public static final double kMidShootRPM = 4200; // 3800
        public static final double kFastShootRPM = 4500; // 4000
        public static final double kLowerShootRPM = 3000;
        public static final double kS = 0.53707;
        public static final double kV = 0.0;
        public static final double kP = 0.008;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kPIDAdjust = 1.225;
    }
public static final class ArmConstants {
        public static final int ARM_MOTOR_ID = 6;
        public static final int ARM_MOTOR_CURRENT_LIMIT = 60;
        public static final double ARM_MOTOR_VOLTAGE_COMP = 10;
        public static final double ARM_SPEED_DOWN = 0.4;
        public static final double ARM_SPEED_UP = -0.4;
        public static final double ARM_HOLD_DOWN = 0.1;
        public static final double ARM_HOLD_UP = -0.15;
        public static final int ARM_LEADER_ID = 50;
    }
    public static final class ClimbConstants {
        public static final int kArmMotor = 10;
        public static final int kHookMotor = 11;
        public static final double kArmMotorSpeed = 1.0;
        public static final double kHookMotorSpeed = 1.0;
        public static final double kArmGearRatio = 48;
        public static final double kHookGearRatio = 36;
        public static final double kArmEncoderMaxValue = 300;
        public static final double kHookEncoderMaxValue = 340;
    }

    public static final class LimelightConstant {
        public static final double kLimelightHeightMeters = Units.inchesToMeters(34);
        public static final double kLimelightAngle = 30;
        public static final double kHubHeightMeters = Units.inchesToMeters(104);
    }

    public static final class OIConstants {
        public static final int kXboxDriverController = 0;
        public static final int kFlightDriverController = 1;
        public static final int kOperatorController = 2;
    }
  public static final class IntakeConstants {
        public static final int kSideIntakeMotor = 5;
        public static final int kFrontIntakeMotor = 6;
        public static final double kIntakeMotorSpeed = 0.5;
        public static final int     INTAKE_MOTOR_CURRENT_LIMIT = 60;
        public static final double INTAKE_MOTOR_VOLTAGE_COMP = 10;
    }
    
    
}
  