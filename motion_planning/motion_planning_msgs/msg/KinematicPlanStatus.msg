# This message contains the definition for the state of a motion plan

# The ID of the request the status message refers to. A value of -1 means that
# replanning is not active
int32 id

# The result of a motion plan: a kinematic path. If the motion plan is
# not succesful, this path has 0 states. If the motion plan is
# succesful, this path contains the minimum number of states (but it
# includes start and goal states) to define the motions for the
# robot. If more intermediate states are needed, linear interpolation
# is to be assumed.
motion_planning_msgs/KinematicPath path

# If the last sent path is valid, this value is set to 1. 
# When this valid is set to 0, the path became invalid and 
# replanning starts. 
byte valid

# The distance to the goal that was actually achieved (heuristic). If the planner did not have enough time,
# it may not return a complete solution return a distance larger than the desired threshold. The user may
# choose to use this path anyway
float64 distance

# If computing the exact path was not possible, but an approximate
# solution is obtained, this will be 1; otherwise, it will be 0
byte approximate

# If the data the planner has when planning is stale, set this flag to 1
# This would happen when map data has not been received recently
byte unsafe

# If the starting state is already in the goal region,
# this is set to 1. Otherwise, it will be set to 0
byte done
