Header header

# Is the controller actively going to a goal?
byte active

# Did the controller find a valid path?
byte valid

# Have we arrived at the goal?
byte done

# If true, the robot is plugged in
byte plugIn

# The desired plug in state
byte desPlugIn

# The goal pose for accessing the plug
std_msgs/Pose2DFloat32 goal