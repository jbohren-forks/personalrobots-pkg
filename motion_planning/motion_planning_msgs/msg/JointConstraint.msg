Header header

# Constrain the position of a joint to be within a certain bound
string joint_name

# the bound to be achieved is [value - toleranceBelow, value + toleranceAbove]
float64[] value
float64[] toleranceAbove
float64[] toleranceBelow
