Header header
# If true, the robot needs to be pluged in, if false, the robot needs to be unplugged.
byte plugIn

# Sets the pose for accessing the plug
std_msgs/Pose2DFloat32 goal

# True to submit a goal, false to recall it
byte enable
