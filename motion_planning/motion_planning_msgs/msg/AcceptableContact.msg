# This message defines a region of space where contacts
# with certain links are allowed, up to a certain depth

# The shape of the bound where contacts are allowed
mapping_msgs/Object bound

# The pose of the box defining the region where contacts are allowed
geometry_msgs/PoseStamped pose

# The set of links that are allowed to touch obstacles
string[] links

# The maximum penetration depth
float64 depth
