package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class goToLocation extends Command {
  
  private Drivebase drivebase;
  private Pose2d goalPose;

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(0.5, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(0.5, 2); 
  private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);
  
  private final ProfiledPIDController xController = new ProfiledPIDController(6, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(6, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController thetaController = new ProfiledPIDController(6, 0, 0, THETA_CONSTRAINTS);

  @SuppressWarnings("unused")
  private double xStart = 0;
  @SuppressWarnings("unused")
  private double yStart = 0;
  @SuppressWarnings("unused")
  private double thetaStart = 0;

  public goToLocation(Drivebase drivebase, Pose2d pose) {
    this.drivebase = drivebase;
    this.goalPose = pose;

    xController.setTolerance(0.02);
    yController.setTolerance(0.02);
    thetaController.setTolerance(Units.degreesToRadians(2));
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

    xStart = robotPose.getX();
    yStart = robotPose.getY();
    thetaStart = robotPose.getRotation().getRadians();

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

  public double xSpeed;
  public double ySpeed;
  public double thetaSpeed;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    Pose2d robotPose = drivebase.getPose();

    xSpeed = -xController.calculate(robotPose.getX());
    ySpeed = -yController.calculate(robotPose.getY());
    thetaSpeed = -thetaController.calculate(robotPose.getRotation().getRadians());

    SmartDashboard.putNumber("current estimated x", robotPose.getX());
    SmartDashboard.putNumber("current estimated y", robotPose.getY());
    SmartDashboard.putNumber("current estimated rotation", robotPose.getRotation().getRadians());

    SmartDashboard.putNumber("x error", robotPose.getX()-goalPose.getX());
    SmartDashboard.putNumber("y error", robotPose.getY()-goalPose.getY());
    SmartDashboard.putNumber("theta error", robotPose.getRotation().getRadians()-goalPose.getRotation().getRadians());

    if (xController.atGoal()) {xSpeed = 0;}
    if (yController.atGoal()) {ySpeed = 0;}
    if (thetaController.atGoal()) {thetaSpeed = 0;}

    SmartDashboard.putNumber("x Speed", xSpeed);
    SmartDashboard.putNumber("y Speed", ySpeed);
    SmartDashboard.putNumber("theta Speed", thetaSpeed);

    if (xSpeed == 0 && ySpeed == 0 && thetaSpeed == 0){this.cancel();}

    drivebase.autoDrive(xSpeed, ySpeed, thetaSpeed, goalPose.getRotation());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finished = false;
    if (xController.atGoal() && yController.atGoal() && thetaController.atGoal()) {
      finished = true;
    } else {
      finished = false;
    }

    return finished;


    //ismael's end thing
    // if (xSpeed == 0 && ySpeed == 0 && thetaSpeed == 0){ 
    //   finished = true;
    // } else if (xSpeed == 0 && ySpeed == 0 || ySpeed == 0 && thetaSpeed == 0 || xSpeed == 0 && thetaSpeed == 0){
    //   if (Math.abs(xSpeed) < 0.175 && Math.abs(ySpeed) < 0.175 && Math.abs(thetaSpeed) < 0.175){
    //     return true;
    //   } else {
    //     return false;
    //   }
    // } else {
    //   finished = false;
    // }
  }
  
}
