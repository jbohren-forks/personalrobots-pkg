Header header
std_msgs/Point32 frame_p1  ## position of the door frame
std_msgs/Point32 frame_p2  ## position of the door frame
std_msgs/Point32 handle    ## position of the center of the door handle, as a point ont the door plane
int16 hinge                ## is the hinge at frame_p1 (=1), frame_p2 (=2) or unknown (=-1)
int16 rot_dir              ## does the door rotate open clockwise (=1), counterclockwise (=2), or unknown (=-1)
float32 angle              ## the rotation of the door, around the Z-axis, where 0 is a closed door
