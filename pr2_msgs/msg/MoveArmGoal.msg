Header header

# Only one of goal_state or goal_constraints is used.  implicit_goal ==
# 1 means use goal_constraints, 0 means use goal_state
byte implicit_goal
# The desired joint state
robot_msgs/KinematicState goal_state
# Goal constraints (implicit definition of goal)
robot_msgs/PoseConstraint[] goal_constraints

# Indicate if the goal is being enabled or disabled
byte enable

# Total timelimit for the controller, if a plan is not
# found by this time, abort. Set it to zero for unlimited.
float64 timeout
