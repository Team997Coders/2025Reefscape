// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Leveraged from PhotonVision Docs:
// https://docs.photonvision.org/en/latest/docs/examples/aimingatatarget.html

package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.vision.Camera;


public class AutoAlignDrive extends Command {

  private final Drivebase drivebase;
  private final Camera camera;
  private final Supplier<double[]> speedXY;
  private final int targetId;
  private double autoAlignYaw = 0.0;

  private final double VISION_TURN_kP = 0.1;
  private final double kMaxAngularSpeed = 10.0;

  /** Creates a new Drive. */
  public AutoAlignDrive(int targetId, Drivebase drivebase, Camera camera, Supplier<double[]> speedXY) {
    this.drivebase = drivebase;
    this.camera = camera;
    this.speedXY = speedXY;
    this.targetId = targetId;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turn = 0.0;
    autoAlignYaw = camera.getAutoAlignYaw(targetId);
    if (autoAlignYaw != -1.0) {
       turn = -1.0 * autoAlignYaw * VISION_TURN_kP * kMaxAngularSpeed;
    }
    var xy = speedXY.get();

    drivebase.defaultDrive(-xy[1], -xy[0], turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
