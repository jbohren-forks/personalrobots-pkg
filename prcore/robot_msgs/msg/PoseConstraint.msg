# This message contains the definition of a motion planning constraint.
# Since there are multiple types of constraints, the 'type' member is used
# to identify the different constraints

byte COMPLETE_POSE=0 # 0 = complete pose is considered
byte ONLY_POSITION=1 # 1 = only pose position is considered
byte ONLY_ORIENTATION=2 # 2 = only pose orientation is considered 

byte type

# The robot link this constraint refers to
string robot_link

# The desired pose of the robot link
std_msgs/Pose3D pose

# the allowed difference (square of euclidian distance) for position
float64 position_distance

# the allowed difference (shortest angular distance) for orientation
float64 orientation_distance
