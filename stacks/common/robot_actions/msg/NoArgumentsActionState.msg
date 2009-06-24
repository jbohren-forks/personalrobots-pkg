# Many actions take no goal, and give no feedback other than the success or failure. For these
# we define a comon message format

Header header

# Status
ActionStatus status

# Goal
std_msgs/Empty goal

# Feedback
std_msgs/Empty feedback