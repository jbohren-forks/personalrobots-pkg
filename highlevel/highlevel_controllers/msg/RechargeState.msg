Header header

# Is the controller actively going to a goal?
byte active

# Did the controller find a valid path?
byte valid

# Have we arrived at the goal?
byte done

#Did the planner give up?
byte aborted

#Was the planner told to stop?
byte preempted

# Actual recharge level
float32 recharge_level

# Target mimimum percentage recharge to achieve
float32 goal_recharge_level

# The goal pose for accessing the plug
std_msgs/Pose2DFloat32 goal_pose
