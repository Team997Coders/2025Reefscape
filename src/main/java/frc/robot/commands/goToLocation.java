package frc.robot.commands;

import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.vision.Camera;

public class goToLocation extends Command {
  
  private Drivebase drivebase;
  private Pose2d pose2d;
  private PathPlannerTrajectory trajectory;
  private int index;
  private Timer time;

  public goToLocation(Drivebase drivebase, Pose2d pose) {
    this.drivebase = drivebase;
    this.pose2d = pose;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses
    // (
    //   this.drivebase.getPose(),
    //   this.pose2d
    // );

    // PathPlannerPath pathplannerPath = new PathPlannerPath(
    //     waypoints,
    //     new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path.,
    //     null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
    //     new GoalEndState(0.0, pose2d.getRotation())); // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    
    // PathfindThenFollowPath path = new PathfindThenFollowPath(
    //   pathplannerPath, 
    //   new PathConstraints(5, 5, 1, 1), 
    //   () -> drivebase.getPose(), 
    //   () -> drivebase.getCurrentSpeeds(), 
    //   (speeds, feedforwards) -> drivebase.driveWithChassisSpeeds(speeds), 
    //   new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
    //                 new PIDConstants(0.1, 0.0, 0.0), // Translation PID constants
    //                 new PIDConstants(0.1, 0.0, 0.0) // Rotation PID constants
    //   ),
    //   drivebase.config, 
    //   () -> (false), 
    //   drivebase);

    //   trajectory = pathplannerPath.generateTrajectory(drivebase.getCurrentSpeeds(), drivebase.getPose().getRotation(), drivebase.config);

    PathfindingCommand pathfindingCommand = new PathfindingCommand(pose2d, 
      new PathConstraints(5, 5, 1, 1), 
      () -> drivebase.getPose(), 
      () -> drivebase.getCurrentSpeeds(), 
      (speeds, feedforwards) -> drivebase.driveWithChassisSpeeds(speeds), 
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                     new PIDConstants(0.1, 0.0, 0.0), // Translation PID constants
                     new PIDConstants(0.1, 0.0, 0.0)), // Rotation PID constants
       drivebase.config, 
       drivebase);

       pathfindingCommand.schedule();
       SmartDashboard.putBoolean("Hey: the path finding command got schedules", pathfindingCommand.isScheduled());
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
