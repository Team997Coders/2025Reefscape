from wpimath.geometry import Pose2d
from frc.robot.utils.april_tag_field_layout import AprilTagFieldLayout
from frc.robot.utils.april_tag_fields import AprilTagFields

def __init__(self, tag_id, side):
    super().__init__()
    self.tag_id = tag_id
    self.side = side
    self.april_tag_field_layout = None

# Called when the command is initially scheduled
def initialize(self):
    self.april_tag_field_layout = AprilTagFieldLayout.load_field(AprilTagFields.k2025ReefscapeWelded)

# Called every time the scheduler runs while the command is scheduled
def execute(self):
    g_pose_2d = self.goal_pose(self.tag_id, self.side)
    print(f"goalPose (inches): {g_pose_2d}")
    g_goal_2d = self.offset_to_goal(g_pose_2d, self.side)
    print(f"Offset Pose (inches): {g_goal_2d}")
    final_pose_2d = self.inch_to_meters(g_goal_2d)
    print(f"Final Pose (meters): {final_pose_2d}")

    # Uncomment and implement this if needed
    # self.drivebase.go_to_location(final_pose_2d)

# Called once the command ends or is interrupted
def end(self, interrupted):
    pass

# Returns true when the command should end
def isFinished(self):
    return True

# Placeholder for goalPose logic
def goal_pose(self, tag_id, side):
    # Implement logic to calculate the goal pose based on tag_id and side
    pass

# Placeholder for offset2Goal logic
def offset_to_goal(self, pose, side):
    # Implement logic to calculate the offset pose
    pass

# Placeholder for inch2meters logic
def inch_to_meters(self, pose):
    # Implement logic to convert pose from inches to meters
    pass