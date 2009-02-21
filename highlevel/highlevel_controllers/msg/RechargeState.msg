Header header

#Status options
byte INACTIVE=0
byte ACTIVE=1
byte ABORTED=2
byte PREEMPTED=3

# Status of the controller = {INACTIVE, ACTIVE, ABORTED, PREEMPTED}
byte status

#Id of the goal
int32 goal_id

#Comment for debug
string comment

# Actual recharge level
float32 recharge_level

# Target mimimum percentage recharge to achieve
float32 goal_recharge_level

# The goal pose for accessing the plug
deprecated_msgs/Pose2DFloat32 goal_pose
