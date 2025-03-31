from wpimath.geometry import Pose2d, Rotation2d

class ReefScoringPose:
    def __init__(self, tag_pose: Pose2d, side: int):
        """
        Initialize the ReefScoringPose class.

        :param tag_pose: The Pose2d of the AprilTag (goal pose).
        :param side: The side of the scoring branch (1 for left, -1 for right).
        """
        self.tag_pose = tag_pose
        self.side = side

    def offset_to_goal(self, goal_pose: Pose2d, side: int) -> Pose2d:
        """
        Calculate the offset pose to align the robot with the scoring branch.

        :param goal_pose: The Pose2d of the goal (AprilTag).
        :param side: The side of the scoring branch (1 for left, -1 for right).
        :return: The Pose2d of the robot in front of the scoring branch.
        """
        print(f"goal rotation: {goal_pose.rotation().degrees()} degrees")

        angle = goal_pose.rotation().radians()

        # 6.5in = 0.1651m
        offset_x = goal_pose.x + (6.5 if side == 1 else -6.5) * 0.0254 * math.sin(angle)  # Offset in meters
        offset_y = goal_pose.y + (6.5 if side == 1 else -6.5) * 0.0254 * math.cos(angle)
        print(f"offset x: {offset_x}, offset y: {offset_y}")

        # Adjust for the robot's width (14.5 inches is half the robot's width + 4in for the bumper)
        # 18.5in = 0.4699m
        offset_x += 18.5 * 0.0254 * math.cos(angle)
        offset_y += 18.5 * 0.0254 * math.sin(angle)

        return Pose2d(offset_x, offset_y, angle)

    def goal_transform(self, goal_pose: Pose2d, side: int) -> Pose2d:
        """
        Apply a transform to the goal pose to align with the scoring branch.

        :param goal_pose: The Pose2d of the goal (AprilTag).
        :param side: The side of the scoring branch (1 for left, -1 for right).
        :return: The transformed Pose2d of the robot.
        """
        front_offset = (14.5 + 4)*0.0254  # Offset for the front of the robot
        left_right_shift = 6.5*0.0254  # Offset for the left or right alignment

        transform_x = front_offset
        transform_y = (1 if side == 1 else -1) * left_right_shift

        # Apply the transform
        transformed_pose = Pose2d(
            goal_pose.x + transform_x,
            goal_pose.y + transform_y,
            goal_pose.rotation().radians()
        )
        return transformed_pose

    def calculate_robot_pose(self):
        """
        Calculate the robot's pose in front of the scoring branch.

        :return: The Pose2d of the robot in front of the scoring branch.
        """
        print(f"Raw Goal Pose: {self.tag_pose}")
        offset_pose = self.offset_to_goal(self.tag_pose, self.side)
        print(f"Offset Pose: {offset_pose}")
        transformed_pose = self.goal_transform(self.tag_pose, self.side)
        print(f"Transformed Pose: {transformed_pose}")
        return transformed_pose


# Example usage
if __name__ == "__main__":
    import math
    from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

    april_tag_field_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded)

    # tag information
    tagId = 21
    # Example usage: Get the pose of a specific AprilTag by its ID
    tag_pose = april_tag_field_layout.getTagPose(tagId).toPose2d()

    if tag_pose is not None:
        print(f"Pose of AprilTag meters for tag {tagId}: {tag_pose}")
        print(f"X: {tag_pose.X()/0.0254}, Y: {tag_pose.Y()/0.0254} inches")
    else:
        print(f"AprilTag {tagId} not found in the field layout.")

    # Example AprilTag pose (in inches)
    # tag_pose = Pose2d(100, 50, Rotation2d.fromDegrees(90))  # Example pose
    side = 1  # 1 for left, -1 for right

    reef_scoring_pose = ReefScoringPose(tag_pose, side)
    robot_pose = reef_scoring_pose.calculate_robot_pose()
    print(f"Robot Pose in front of scoring branch: {robot_pose}")