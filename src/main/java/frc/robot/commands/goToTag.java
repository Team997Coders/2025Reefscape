package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class goToTag extends Command {
  private int tagId;
  private int side;
  private AprilTagFieldLayout aprilTagFieldLayout;

  public goToTag(int TagId, int side) {
    this.tagId = TagId;
    this.side = side;
  }

  private Pose2d goalPose(int TagId, int side) {
    Pose3d tagInFieldFrame;

    
    if (aprilTagFieldLayout.getTagPose(tagId).isPresent()) // margin < 20 seems bad > 140 are good maybe > 50 a limit?
    {
      tagInFieldFrame = aprilTagFieldLayout.getTagPose(tagId).get();
      System.out.println("tagInFieldFrame: " + tagInFieldFrame);
      Pose2d tempPose2d = new Pose2d(Units.metersToInches(tagInFieldFrame.getX()), 
          Units.metersToInches(tagInFieldFrame.getY()),
          tagInFieldFrame.getRotation().toRotation2d());
      System.out.println("Tag: " + tagId +", tagInFieldFrame: " + tempPose2d);
      return tempPose2d;
    } else {
      System.out.println("bad id " + tagId);
      return null;
    }
  }

  //
  private Pose2d offset2Goal(Pose2d goalPose2d, int side) {
    // offset position to the left or right to align with the goal branch.
    double angle = goalPose2d.getRotation().getRadians();
    double offsetX = goalPose2d.getX() + (side == 1 ? 6.5 : -6.5) * Math.sin(angle); // 6.5 is the offset in inches
    double offsetY = goalPose2d.getY() + (side == 1 ? 6.5 : -6.5) * Math.cos(angle); // 0 is the offset in inches
    System.out.println("offsetX: " + offsetX + ", offsetY: " + offsetY + ", angle: " + angle);

    offsetX += 14.5 * Math.cos(angle); // 14.5 half the width of the robot
    offsetY += 14.5 * Math.sin(angle); //

    return new Pose2d(offsetX, offsetY, goalPose2d.getRotation());
  }

  //
  private Pose2d inch2meters(Pose2d in_pose2d) {
    // convert inches to meters
    return new Pose2d(Units.inchesToMeters(in_pose2d.getX()), Units.inchesToMeters(in_pose2d.getY()),
        in_pose2d.getRotation());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d gPose2d = goalPose(tagId, side);
    System.out.println("goalPose (inches): " + gPose2d);
    Pose2d gGoal2d = offset2Goal(gPose2d, side);
    System.out.println("Offset Pose (inches): " + gGoal2d);
    Pose2d finalPose2d = inch2meters(gGoal2d);
    System.out.println("Final Pose (meters): " + finalPose2d);

    //new goToLocation(drivebase, finalPose2d);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
