package frc.robot.commands;

import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.vision.Camera;

public class goToTag extends Command {
  
  private Drivebase drivebase;
  private Camera frontCamera;
  private Double radius;
  private Command currentPath;

  public goToTag(Drivebase drivebase, Camera frontCamera, Double radius) {
    this.drivebase = drivebase;
    this.frontCamera = frontCamera;
    this.radius = radius;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (frontCamera.hasTarget())
    {
    //double theta = frontCamera.robot_to_tag(drivebase).getAngle().getRadians();
    //Transform2d tagOffset = new Transform2d(this.radius*Math.cos(theta), this.radius*Math.sin(theta), new Rotation2d(0));
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses
    (
      this.drivebase.getPose(),
      this.frontCamera.get_tag_pose2d()//.plus(tagOffset)
    );

    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path.,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
        new GoalEndState(0.0, Rotation2d.fromDegrees(-90))); // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    AutoBuilder.followPath(path).schedule();
    } else 
    {
      this.cancel();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
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
