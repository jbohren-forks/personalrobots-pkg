To shut down debugging display
set DISPLAY to false in stereo_blob_tracker.cpp
set BLOBNEARCENTER to true if the head tracks the blob and the blob is alway
near the center.

start botherder or master


run the following command to start the camera server, who sends out images
and calibaration parameters


roslaunch videre.xml

run the following command to start the tracker:

./bin/stereo_blob_tracker images:=videre/images calparams:=videre/cal_params points:=videre/tracked_points

from the tracker window, select the object you want to track by draging
a rectangle area with the mouse. Be sure that
your selection area (rectangle) is completely within the blob.

The 3D points are publish in camera frame, right hand system
x to the right, y downward and z forward, the unit is meter.

run the following command to verify 3d points are being published
bin/stereo_blob_tracker_listener points:=videre/tracked_points

NOTE: because of the white balancing issue, please use a bluish or cyan object to track.


