# This is a meta message that stores data from multiple sensors along
# with the current mechanism state of the robot.  Having this meta packet
# is extremely useful when trying to do kinematic calibration.

Header header

# Raw Stereocam Sensor Data
image_msgs/RawStereo raw_stereo

# Mechanism State - Stores the robots joint angles
robot_msgs/MechanismState mechanism_state

# PointCloud generated from the tilt laser
robot_msgs/PointCloud laser_cloud

# Data that we captured from phasespace
mocap_msgs/MocapSnapshot mocap_snapshot

# Data from multiple monocular cameras
image_msgs/Image[] image
image_msgs/CamInfo[] cam_info
