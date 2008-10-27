To shut down debugging display
set DISPLAY to false in stereo_blob_tracker.cpp
set BLOBNEARCENTER to true if the head tracks the blob and the blob is alway
near the center.

start botherder or master


run the following command to start the camera server, who sends out images
and calibaration parameters

roscd bread_board
roslaunch videre.xml

run the following command to start the tracker:

roscd stereo_blob_tracker
roslaunch videre.xml

if you prefer to run the gui remotely (for example, outside of the robot),
set the #define variable REMOTE_GUI to true in stereo_blob_tracker.cpp to true and 
run

roslaunch videre.xml  on the robot
roslaunch gui.xml     somewhere else. Make sure you set your ROS_MASTER_URI to point to where you run your roscore.

from the tracker window, select the object you want to track by draging
a rectangle area with the mouse. Be sure that
your selection area (rectangle) is completely within the blob.

The 3D points are publish in camera frame, right hand system
x to the right, y downward and z forward, the unit is meter.

run the following command to verify 3d points are being published, selection box and
tracked box are being published.

roslaunch listener.xml


NOTE: because of the white balancing issue, please use a bluish or cyan object to track.


