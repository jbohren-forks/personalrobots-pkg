Header header

#Status options
byte INACTIVE=0
byte ACTIVE=1
byte ABORTED=2
byte PREEMPTED=3

# Status of the controller = {INACTIVE, ACTIVE, ABORTED, PREEMPTED}
byte status

#Id of the goal
int32 goal_id

#Comment for debug
string comment

#Current arm configuration
robot_msgs/JointState[] configuration
#Goal arm configuration
robot_msgs/JointState[] goal
