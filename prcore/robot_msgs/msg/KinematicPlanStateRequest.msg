# This message contains the definition for a request to the motion
# planner

# Parameters for the state space 
robot_msgs/KinematicSpaceParameters params

# The starting state for the robot. This is the complete state of the
# robot, all joint values, everything that needs to be specified to
# completely define where each part of the robot is in space. The
# meaning for each element in the state vector can be extracted from
# viewing the output of the robot model construction (the kinematic
# model constructed from the parsed URDF model) in verbose mode
robot_msgs/KinematicState start_state


# The goal state for the model to plan for. The dimension of this
# state must be equal to the dimension of the state space
# characterizing the model (group) to plan for.
robot_msgs/KinematicState goal_state

# No state in the produced motion plan will violate these constraints
robot_msgs/KinematicConstraints constraints

# The number of times this plan is to be computed. Shortest solution
# will be reported.
int32 times

# Boolean flag: if true, the returned path will contain interpolated
# states as well
byte interpolate

# The maximum amount of time the motion planner is allowed to plan for
float64 allowed_time


# The threshold (distance) that is allowed between the returned
# solution and the actual goal
float64 threshold
