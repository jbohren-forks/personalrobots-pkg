# This message contains the definition of a motion planning constraint.
# Since there are multiple types of constraints, the 'type' member is used
# to identify the different constraints


# Value that identifies the type of constraint
byte type


##########################################
# For constraints of type 0              #
# (position of center of a certain body) #
##########################################

# The name of the body (link) for which the constraint is defined
string body

# The position of the body
std_msgs/Point3DFloat64 position

# The tolerance for this position
float64 tolerance
