// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import swervelib.SwerveModuleConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class ElevatorConstants{
    public static final int leftSparkMaxID = 9; //CAN
    public static final int rightSparkMaxID = 10;

    public static final int climberServoID = 9;
    public static final double climberAngle1 = 0;
    public static final double climberAngle2 = 0;
    
    public static final double climberEncoderPosition = 0;

    public static final boolean leftSparkMaxInverted = false;
    public static final boolean rightSparkMaxInverted = true;

    public static final int bottomSwitchID = 0; //DIO

    public static final int atTargetOffset = 0; //The encoder ticks to determine whether the elevator is at the target position

    // elevator travel is about 30in
    public static final class SetpointRotations {
      public static final double MAX = 10;
      public static final double DOWN = 0;
      public static final double SOURCE = 0;
      public static final double L1 = 0;
      public static final double L2 = 0;
      public static final double L3 = 0;
      public static final double L4 = 0;
    }

    public static final class PID {
      public static final double kP = 0.1;
      public static final double kI = 0;
      public static final double kD = 0;
    }

    public static final class FeedForward{
      public static final double kS = 0;
      public static final double kG = 0;
      public static final double kV = 0;
    }

    public static final class SIM {
      public static final double kElevatorKp = 0.1;
      public static final double kElevatorKi = 0;
      public static final double kElevatorKd = 0;

      public static final double kElevatorkS = 0.0; // volts (V)
      public static final double kElevatorkG = 0.762; // volts (V)
      public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
      public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))

      public static final double kElevatorGearing = 25.0;
      public static final double kElevatorDrumDia = Units.inchesToMeters(1.76);
      public static final double kCarriageMass = Units.lbsToKilograms(20.0); // kg

      public static final double kSetpointMeters = 0.75;
      // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
      public static final double kMinElevatorHeightMeters = 0.0;
      public static final double kMaxElevatorHeightMeters = 1.25;

      // distance per pulse = (distance per revolution) / (pulses per revolution)
      //  = (Pi * D) / ppr
      // REV NEo motor encoders have 42 counts per revolution
      // Note:  Assuming that the max motion of the elevator chain
      //      is 34inches (which is 36 height minus the overlap).
      //      The gear stack is:
      //        motor->motor encoder->25:1 gearbox->14tooth sprocket->chain
      //      Sprocket is from Thrifty-bot kit: 25 Chain 22t 1/2" Hex Sprocket = 1.76"
      // This implies that the for a total travel of 34 inches => 6.15 rotations.
      public static final double kElevatorEncoderDistPerPulse =
        Math.PI * kElevatorDrumDia / 42;  // 0.131 inches/encoder tic
    }

    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double kMinElevatorHeightMeters = 0.0;
    public static final double kMaxElevatorHeightMeters = 1.25;

    public static final double defaultManualOutput = 2;
  }

  public static final class Coral {
    public static final int leftMotorID = 11; //CAN
    public static final int rightMotorID = 12;
    public static final int beamBrake1ID = 1; //DIO
    public static final int beamBrake2ID = 2;
    public static final double motorSpeedOutTake = 0.5;
    public static final double motorSpeedIntake = 0.5;
    public static final boolean leftMotorInverted = true;
    public static final boolean rightMotorInverted = false;
  }
  
  public static final class Algae{
    public static final int spinnyMotorID = 13; //CAN
    public static final int spinnyMotorConfig = 15;
    public static final int proximitySensorID = 9; //DIO
    public static final double motorSpin = 0.8;
  }
  
  public static final class DriveConstants {
    public static final double deadband = 0.08;
    public static final int currentLimit = 40;
    public static final double slewRate = 20; // lower number for higher center of mass

    public static final class SwervePID {
      public static final double p = 0.01;
      public static final double i = 0;
      public static final double d = 0;
    }

    public static final class SwerveModules {

      // Front Left Module
      public static final SwerveModuleConfig frontLeft = new SwerveModuleConfig(
          5,
          6,
          true,
          true,
          false,
          1,
          //.462
          0);

      // Front Right
      public static final SwerveModuleConfig frontRight= new SwerveModuleConfig(
          3,
          4,
          true,
          true,
          false,
          1,
          //0
          0);

      // Back Right
      public static final SwerveModuleConfig backRight = new SwerveModuleConfig(
          1,
          2,
          true,
          true,
          false,
          1,
          //.759
          0);

      // Back Left
      public static final SwerveModuleConfig backLeft = new SwerveModuleConfig(
          7,
          8,
          true,
          true,
          false,
          1,
          //.158 // 0 to 1
          0
      );
    }

    public static final class ModuleLocations {
      public static final double dist = Units.inchesToMeters(14.5);
      public static final double robotRaduius = Math.sqrt(2 * Math.pow(dist, 2));
      public static final Translation2d frontLeft = new Translation2d(dist, dist);
      public static final Translation2d frontRight = new Translation2d(dist, -dist);
      public static final Translation2d backLeft = new Translation2d(-dist, dist);
      public static final Translation2d backRight = new Translation2d(-dist, -dist);
    }
  }

  public static final class AutoConstants {
    public static final class XPID {
      public static final double p = 1.5;
      public static final double i = 0;
      public static final double d = 0;
    }

    public static final class YPID {
      public static final double p = 1.5;
      public static final double i = 0;
      public static final double d = 0;
    }

    public static final class RPID {
      public static final double p = 0.0015;
      public static final double i = 0;
      public static final double d = 0.0002;
    }

    public static final int medianFilter = 5;
  }

  public static final class PathPlannerConstants {
    public static final class TranslationPID {
      public static final double p = 5;
      public static final double i = 0;
      public static final double d = 0;
    }

    public static final class RotationPID {
      public static final double p = 6;
      public static final double i = 0;
      public static final double d = 0;
    }
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class CANdleConstants {
    public static final int id = 50;
    public static final int ledCount = 50;
  }
}
