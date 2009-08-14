# This mesage defines what objects are attached to a link. 

Header header

# The object will be attached with a fixed joint to this link
string link_name

# The definition of the objects
Object[] objects

# The poses of the objects in the header frame. Each pose is transformed to the frame of 
# the link we are attaching to and the resulting transform is considered fixed
geometry_msgs/Pose[] poses

# The set of links that the attached object is allowed to touch
string[] touch_links
