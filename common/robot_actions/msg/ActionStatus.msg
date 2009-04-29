# This message defines the expected format for robot action status messages
# Embed this in the feedback state message of robot actions

# The action is inactive, and its status with respect to a prior request is undefined.
byte UNDEFINED=0

# The action has successfuly completed and is now inactive
byte SUCCESS=1

# The action has failed and given up. It is now inactive
byte ABORTED=2

# The action has been preempted. It is now inactive
byte PREEMPTED=3

# The action is active to accomplish a requested goal
byte ACTIVE=4

# Status of the controller = {UNDEFINED, SUCCESS, ABORTED, PREEMPTED, ACTIVE}
byte value

#Comment for debug
string comment