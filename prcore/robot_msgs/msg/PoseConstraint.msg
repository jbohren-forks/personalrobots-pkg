# This message contains the definition of a motion planning constraint.
# Since there are multiple types of constraints, the 'type' member is used
# to identify the different constraints


int32 POSITION_XYZ=1 #  only x,y,z of position is considered
int32 POSITION_XY=2  #  only x,y of position is considered
int32 POSITION_XZ=3  #  only x,z of position is considered
int32 POSITION_YZ=4  #  only y,z of position is considered
int32 POSITION_X=5   #  only x of position is considered
int32 POSITION_Y=6   #  only y of position is considered
int32 POSITION_Z=7   #  only z of position is considered

# the next values can be combined with one of the above, so they are offset by
# 256, so we can use bit operations on them

int32 ORIENTATION_RPY=256  # only roll, pitch, yaw of orientation is considered 
int32 ORIENTATION_RY=512   # only roll, yaw of orientation is considered 
int32 ORIENTATION_RP=768   # only roll, yaw of orientation is considered 
int32 ORIENTATION_PY=1024   # only roll, yaw of orientation is considered 
int32 ORIENTATION_R=1280    # only roll, yaw of orientation is considered 
int32 ORIENTATION_P=1536    # only roll, yaw of orientation is considered 
int32 ORIENTATION_Y=1792    # only roll, yaw of orientation is considered 

int32 type

# The robot link this constraint refers to
string robot_link

# The desired pose of the robot link (in the robot frame)
float64 x
float64 y
float64 z
float64 roll   # around Z axis
float64 pitch  # around X axis
float64 yaw    # around Y axis

# the allowed difference (square of euclidian distance) for position
float64 position_distance

# the allowed difference (shortest angular distance, in radians) for orientation
float64 orientation_distance

# a factor that gets multiplied to the orientation when computing the
# distance to the goal.
# Distance_to_goal = position_distance + orientation_importance * orientation_distance
float64 orientation_importance
