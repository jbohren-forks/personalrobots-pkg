Header header
# If true, the robot needs to be pluged in, if false, the robot needs to be unplugged.
byte plugIn

# Sets the target recharge_level
float32 recharge_level

# Sets the pose for accessing the plug
std_msgs/Pose2DFloat32 pose

# True to submit a goal, false to recall it
byte enable
