#A 2-D grid score map

#in meters/cell
float32 scale

uint32 size_x
uint32 size_y

#bottom left corner of the map in world space
std_msgs/Pose2DFloat32 origin

ScoreMapCell2D[] data
