package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.exceptions.noNextAction;
import frc.robot.exceptions.outOfBounds;

public class getTagOffset{
  private int tagId;
  private int side;
  private AprilTagFieldLayout aprilTagFieldLayout;

  // front back offsert is half of length of the robot + the width of the bumper
  //     = 29in / 2 + 4in = 18.5in
  // left right offset is the distance from the center of the tag to the left or right side scoring branch
  //     = 6.5in
  private static final double frontOffset = Units.inchesToMeters(29.0/2.0 + 4.0);
  private static final double leftRightShift = Units.inchesToMeters(6.5);

  // Drive to the scoring branch of a tag.
  public getTagOffset(int TagId, int side) {
    this.tagId = TagId;
    this.side = side;

    // FIRST provided layout of the AprilTags on the field
    // The bottom left corner of the field:
    //   X = 0: Bottom left of the blue alliance wall, X is increasing to the right
    //   Y = 0: Bottom left of the red alliance wall, Y is increasing up
    // NOTE: The measurements on the drawing are in inches, but the file provided returns meters
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  }

  /**
   * @param TagId The id of the Target tag to get the pose
   * @return Pose2d of the tag in the field frame
   *
   * Get the pose of the target tag
   */
  private Pose2d goalTagPose(int TagId) throws outOfBounds{
    Pose2d tagInFieldFrame;
    
    if (aprilTagFieldLayout.getTagPose(tagId).isPresent()) // margin < 20 seems bad > 140 are good maybe > 50 a limit?
    {
      tagInFieldFrame = aprilTagFieldLayout.getTagPose(tagId).get().toPose2d();
      return tagInFieldFrame;
    } else {
      throw new outOfBounds("Hey there is no tag with this id");
    }
  }

  /**
   * 
   * @param goalPose2d Pose of the tag in the field frame
   * @param side Which side (left or right) to offset the robot to align with the target branch (1 = left, -1 = right)
   * @return targetPose Final pose of the robot to drive to
   * 
   * Create a transform for the goal pose to align with the target brandh and offset for the length 
   * of the robot and width of the bumpers
  */

  // Transform the goal pose using direct geometry
  private Pose2d offset2Goal(Pose2d goalPose2d, int side) {
    // offset position to the left or right to align with the goal branch.
    double angle = goalPose2d.getRotation().getRadians();
    double offsetX = goalPose2d.getX() + (side == -1 ? 6.5 : -6.5) * Math.sin(angle) * 0.0254; // 6.5 is the offset in inches
    double offsetY = goalPose2d.getY() + (side == -1 ? 6.5 : -6.5) * Math.cos(angle) * 0.0254; // 0 is the offset in inches
    System.out.println("offsetX: " + offsetX + ", offsetY: " + offsetY + ", angle: " + angle);

    offsetX += 11.5 * Math.cos(angle) * 0.0254; // 14.5 half the width of the robot
    offsetY += 11.5 * Math.sin(angle) * 0.0254; //

    return new Pose2d(offsetX, offsetY, new Rotation2d(goalPose2d.getRotation().getRadians() + Math.PI));
  }

  // Called when the command is initially scheduled.
  public Pose2d getTargetLocation() throws noNextAction {
    Pose2d gPose2d;
    try {
      gPose2d = goalTagPose(tagId);
      Pose2d finalPose2d = offset2Goal(gPose2d, side);
      System.out.println("Offset Pose: " + finalPose2d);
      return finalPose2d;
    } catch (outOfBounds e) {
      e.printStackTrace();
    }
    throw new noNextAction("there is no pose something went wrong");
  }
}
