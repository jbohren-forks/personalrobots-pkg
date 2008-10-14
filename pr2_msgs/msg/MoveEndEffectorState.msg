Header header
#Is the planner/controller actively trying to achieve a goal
byte active
#Did the planner/controller find a valid way to achieve the goal
byte valid
#Did we actually successfully arrive at the goal?
byte done
#Current arm configuration
robot_msgs/JointState[] configuration
#Goal arm configuration
robot_msgs/JointState[] goal
