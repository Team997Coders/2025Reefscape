// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package swervelib;

/** Add your docs here. */
public class SwerveModuleConfig {
  public final int driveMotorId;
  public final int angleMotorId;
  public final boolean driveMotorReversed;
  public final boolean angleMotorReversed;
  public final boolean angleEncoderReversed;
  public final double angleEncoderConversionFactor;
  public final double angleEncoderOffset;

  public SwerveModuleConfig(int driveMotorId, int angleMotorId, boolean driveMotorReversed,
            boolean angleMotorReversed, boolean angleEncoderReversed, double angleEncoderConversionFactor,
            double angleEncoderOffset) {
      this.driveMotorId = driveMotorId;
      this.angleMotorId = angleMotorId;
      this.driveMotorReversed = driveMotorReversed;
      this.angleMotorReversed = angleMotorReversed;
      this.angleEncoderReversed = angleEncoderReversed;
      this.angleEncoderOffset = angleEncoderOffset;
      this.angleEncoderConversionFactor = angleEncoderConversionFactor;
  }
}
