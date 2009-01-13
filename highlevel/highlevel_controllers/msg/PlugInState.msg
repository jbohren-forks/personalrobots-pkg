Header header
# Is the plug in system waiting for a human?
byte active
# Always true
byte valid
# Have we been plugged in or unplugged?
byte done
# Are we waiting to be plugged in or unplugged. 1 if plug in.
byte goal
#Did the planner give up?
byte aborted
#Was the planner told to stop?
byte preempted
# Are we plugged in or unplugged. 1 if plug in.
byte state
