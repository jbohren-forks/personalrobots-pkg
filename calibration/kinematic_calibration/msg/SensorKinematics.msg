# This is a meta message that stores data from multiple sensors along
# with the current mechanism state of the robot.  Having this meta packet
# is extremely useful when trying to do kinematic calibration.

Header header

# Stereocam Sensor Data
image_msgs/Image left_image
image_msgs/Image right_image
image_msgs/Image disparity_image
image_msgs/StereoInfo stereo_info
image_msgs/CamInfo left_info
image_msgs/CamInfo right_info

# Mechanism State - Stores the robots joint angles
robot_msgs/MechanismState mechanism_state

# PointCloud generated from the tilt laser
std_msgs/PointCloud laser_cloud

# Data that we captured from phasespace
phase_space/PhaseSpaceSnapshot phase_space_snapshot
