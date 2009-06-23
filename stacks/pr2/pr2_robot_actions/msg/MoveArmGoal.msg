# The goal state for the model to plan for. The state is not necessarily explicit.
# The goal is considered achieved if all the constraints are satisfied.
# An explicit state can be specified by setting joint constraints to a specific value.
motion_planning_msgs/KinematicConstraints goal_constraints

# The set of constraints that need to be satisfied by the entire solution path.
motion_planning_msgs/KinematicConstraints path_constraints
