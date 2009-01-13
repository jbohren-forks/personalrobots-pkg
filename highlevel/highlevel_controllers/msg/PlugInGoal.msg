Header header
#If true, the robot needs to be pluged in, if false, the robot needs to be unplugged.
byte goal
#True to submit a goal, false to recall it
byte enable

# Timelimit for the controller, if a plan is not
# found by this time, abort. Set it to zero for unlimited.
float64 timeout
