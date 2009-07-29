# This is a meta message that stores data from multiple sensors along
# with the current mechanism state of the robot.  Having this meta packet
# is extremely useful when trying to do kinematic calibration.

Header header

# Stereocam Sensor Data
sensor_msgs/Image left_image
sensor_msgs/Image right_image
sensor_msgs/Image disparity_image
sensor_msgs/StereoInfo stereo_info
sensor_msgs/CamInfo left_info
sensor_msgs/CamInfo right_info

# Mechanism State - Stores the robots joint angles
mechanism_msgs/MechanismState mechanism_state

# PointCloud generated from the tilt laser
sensor_msgs/PointCloud laser_cloud

# Data that we captured from phasespace
mocap_msgs/MocapSnapshot phase_space_snapshot
