# The definition of a kinematic path.  
Header header

# The full state of the robot at the start of the path
# As described by KinematicModel::StateParams
KinematicState start_state

# The model for which this plan was computed
string model_id

# The joint names, in the same order as the values in the state
# and the same order reported by KinematicModel::StateParams
string[] names

# A list of states, each with the dimension of the requested model.
# The dimension of the model may be different from the number of joints
# (when there are joints that use more than one parameter)
KinematicState[] states

# Same length as the list of states, the time at which a state should
# be along the path
float64[] times
