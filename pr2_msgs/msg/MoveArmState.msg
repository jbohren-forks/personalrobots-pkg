Header header
#Is the planner/controller actively trying to achieve a goal
byte active
#Did the planner/controller find a valid way to achieve the goal
byte valid
#Did we actually successfully arrive at the goal?
byte done
#Did the planner give up?
byte aborted
#Was the planner told to stop?
byte preempted
#Current arm configuration
robot_msgs/KinematicState configuration
# Only one of goal_state or goal_constraints is used.  implicit_goal ==
# 1 means use goal_constraints, 0 means use goal_state
byte implicit_goal
# The desired joint state
robot_msgs/KinematicState goal_state
# Goal constraints (implicit definition of goal)
robot_msgs/PoseConstraint[] goal_constraints
