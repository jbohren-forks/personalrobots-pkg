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
robot_msgs/KinematicPath path


# The threshold that was actually achieved. If the planner did not have enough time,
# it may return a distance larger than the desired threshold. The user may choose to 
# discard the path
float64 distance


# If the starting state was already in the goal region,
# this is set to 1. Otherwise, it will be set to 0
byte done


# If computing the path was not possible, this will be set to 0
# otherwise, it will be 1
byte valid

# If the solution only gets close to the goal but does not reach it
# this is set to 1. Otherwise, it will be set to 0
byte approximate

# If the data the planner has when planning is stale, set this flag to 1
byte unsafe
