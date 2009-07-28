# This message contains the definition of a motion planning constraint.
# Since there are multiple types of constraints, the 'type' member is used
# to identify the different constraints


# Constants that represent possible values for type. Each degree of freedom
# can be constrained individually 

int32 POSITION_X=1   #  only x of position is considered
int32 POSITION_Y=2   #  only y of position is considered
int32 POSITION_Z=4   #  only z of position is considered
    
int32 ORIENTATION_R=8     # only roll of orientation is considered 
int32 ORIENTATION_P=16    # only pitch of orientation is considered 
int32 ORIENTATION_Y=32    # only yaw of orientation is considered 

int32 type

# The robot link this constraint refers to
string link_name

# The desired pose of the robot link
robot_msgs/PoseStamped pose

# The acceptable tolerance
robot_msgs/Point position_tolerance_above
robot_msgs/Point position_tolerance_below

# The acceptable tolerance (roll pitch yaw)
robot_msgs/Point orientation_tolerance_above
robot_msgs/Point orientation_tolerance_below

# The planner may internally compute a distance from current state to goal pose.
# This is done with a weighted sum such as:
# Distance_to_goal = position_distance + orientation_importance * orientation_distance;
# orientation_importance can be used to tell the planner which component is more important
# (this only makes a difference when approximate solutions are found)
# If you do not care about this value, simply set it to 1.0
# Planners should use square of 2-norm for position distance and shortest angular distance
# for orientations (roll, pitch yaw)
float64 orientation_importance
