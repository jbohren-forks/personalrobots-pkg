Header header
#Is the planner/controller actively trying to achieve a goal
byte active
#Did the planner/controller find a valid way to achieve the goal
byte valid
#Did we actually successfully arrive at the goal?
byte done
#Current arm configuration
ArmConfiguration configuration
#Goal arm configuration
ArmConfiguration goal
