# The definition of a kinematic path.  the
# joint names the states correspond to and the times at which 
# each state should be along the path

# The full state of the robot at the start of the path
# As described by KinematicModel::StateParams
KinematicState start_state

# The joint names, in the same order as the values in the state
string[] names

# A list of states, each with the dimension of the requested model 
KinematicState[] states

# Same length as the list of states, the time at which a state should
# be along the path
float64[] times
