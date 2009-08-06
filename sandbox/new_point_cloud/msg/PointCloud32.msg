Header header

# 2D structure of the point cloud. If the cloud is unordered,
# height is 1 and width is the length of the point cloud.
uint32 width
uint32 height

new_point_cloud/Point32[] points

sensor_msgs/ChannelFloat32[] channels
