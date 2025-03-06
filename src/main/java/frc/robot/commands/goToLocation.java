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

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.vision.Camera;

public class goToLocation extends Command {
  
  private Drivebase drivebase;
  private Pose2d goalPose;

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1); 
  private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);
  
  private final ProfiledPIDController xController = new ProfiledPIDController(8, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(8, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController thetaController = new ProfiledPIDController(6, 0, 0, THETA_CONSTRAINTS);

  public goToLocation(Drivebase drivebase, Pose2d pose) {
    this.drivebase = drivebase;
    this.goalPose = pose;

    xController.setTolerance(0.05);
    yController.setTolerance(0.05);
    thetaController.setTolerance(Units.degreesToRadians(5));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d robotPose = drivebase.getPose();
    
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
    thetaController.reset(robotPose.getRotation().getRadians());

    xController.setGoal(goalPose.getX());
    yController.setGoal(goalPose.getY());
    thetaController.setGoal(goalPose.getRotation().getRadians());

    SmartDashboard.putNumber("robot Start Pose x", robotPose.getX());
    SmartDashboard.putNumber("robot Start Pose y", robotPose.getY());
    SmartDashboard.putNumber("robot Start Pose theta", robotPose.getRotation().getRadians());

    SmartDashboard.putNumber("xController reset location", robotPose.getX());
    SmartDashboard.putNumber("yController reset location", robotPose.getY());
    SmartDashboard.putNumber("thetaController reset location", robotPose.getRotation().getRadians());

    SmartDashboard.putNumber("robot Goal Pose x", goalPose.getX());
    SmartDashboard.putNumber("robot Goal Pose y", goalPose.getY());
    SmartDashboard.putNumber("robot Goal Pose theta", goalPose.getRotation().getRadians());

    SmartDashboard.putNumber("xController goal location", goalPose.getX());
    SmartDashboard.putNumber("yController goal location", goalPose.getY());
    SmartDashboard.putNumber("thetaController goal location", goalPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    Pose2d robotPose = drivebase.getPose();

    var xSpeed = -xController.calculate(robotPose.getX());
    var ySpeed = -yController.calculate(robotPose.getY());
    var thetaSpeed = -thetaController.calculate(robotPose.getRotation().getRadians());

    SmartDashboard.putNumber("current rotation", robotPose.getRotation().getRadians());

    if (xController.atGoal()) {xSpeed = 0;}
    if (yController.atGoal()) {ySpeed = 0;}
    if (thetaController.atGoal()) {thetaSpeed = 0;}

    SmartDashboard.putNumber("x Speed", xSpeed);
    SmartDashboard.putNumber("y Speed", ySpeed);
    SmartDashboard.putNumber("theta Speed", thetaSpeed);

    if (xSpeed == 0 && ySpeed == 0 && thetaSpeed == 0){this.cancel();}

    drivebase.defaultDrive(xSpeed, ySpeed, thetaSpeed);
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
