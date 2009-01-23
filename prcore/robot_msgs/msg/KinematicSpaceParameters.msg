# This message contains a set of parameters useful in setting up a
# kinematic space for planning

# The model (defined in the robot description as a group of links)
# for which the motion planner should plan for
string model_id


# The name of the motion planner to use. If no name is specified,
# a default motion planner will be used
string planner_id


# A string that identifies which distance metric the planner should use
string distance_metric

# Lower coordinate for a box defining the volume in the workspace the
# motion planner is active in.  If planning in 2D, only first 2 values
# (x, y) of the points are used.
std_msgs/Position volumeMin


# Higher coordinate for the box described above
std_msgs/Position volumeMax
