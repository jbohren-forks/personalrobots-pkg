# Defines a navigation location. This should be part of the nav stack package
Header header

# The planar pose to accomplish
float32 x
float32 y
float32 z
float32 th

# A polyline that cannot be crossed
robot_msgs/Polyline2D boundary
