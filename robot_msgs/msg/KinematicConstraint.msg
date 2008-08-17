# This message contains the definition of a motion planning constraint.
# Since there are multiple types of constraints, the 'type' member is used
# to identify the different constraints

# 0 = complete pose is considered
# 1 = only pose position is considered
# 2 = only pose orientation is considered 
byte type

# The robot link this constraint refers to
string robot_link

# The desired pose of the robot link
std_msgs/Pose3DFloat64 pose

# the allowed difference (euclidian distance) for position
float64 position_distance

# the allowed difference (shortest angular distance) for orientation
float64 orientation_distance
