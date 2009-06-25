# Many actions take no goal, and give no feedback other than the success or failure. For these
# we define a comon message format

Header header

# Status
robot_actions/ActionStatus status

# Goal
TextGoal goal

# Feedback
robot_msgs/Path feedback
