Header header
std_msgs/Point32 frame_p1  ## position of the door frame
std_msgs/Point32 frame_p2  ## position of the door frame
std_msgs/Point32 door_p1   ## Ground plane projection of a point on the plane of the door 
std_msgs/Point32 door_p2   ## Ground plane projection of a point on the plane of the door
std_msgs/Point32 handle    ## Position of the door handle w.r.t a frame defined at the axis of the door and on the ground plane with Z axis pointing up and X axis along the door
int16 hinge                ## is the hinge at frame_p1 (=1), frame_p2 (=2) or unknown (=-1)
int16 rot_dir              ## does the door rotate open clockwise (=1), counterclockwise (=2), or unknown (=-1)
