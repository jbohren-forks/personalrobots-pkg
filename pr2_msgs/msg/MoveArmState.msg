Header header
#Is the planner/controller actively trying to achieve a goal
byte active
#Did the planner/controller find a valid way to achieve the goal
byte valid
#Did we actually successfully arrive at the goal?
byte success
#Current arm location
float64[] actual_arm_state
#Goal arm location
float64[] desired_arm_state
