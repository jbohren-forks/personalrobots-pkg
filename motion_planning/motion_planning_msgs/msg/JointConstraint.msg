Header header

# Constrain the position of a joint to be within a certain bound
string joint_name

# the bound to be achieved is [value - tolerance_below, value + tolerance_above]
float64[] value
float64[] tolerance_above
float64[] tolerance_below
