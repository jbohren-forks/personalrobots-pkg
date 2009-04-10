# This action is the inverse of the MoveAndGrasp action. It will stow the
# Plug on the base and move the arm clear
Header header

# Status
ActionStatus status

# Goal
robot_msgs/PlugStow goal

# Feedback
std_msgs/Empty feedback
