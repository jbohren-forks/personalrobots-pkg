///
///
///   Calibration between stereo camera (videre) and laser range finder (hokuyo),
///
///

DATA PREPARATION:



ddd-J.xml  (lrf points, e.g. 007-J.xml)
ddd-L.png  (rectified image from the left camera)
ddd-R.png  (rectified image from the right camera)

use calib_converter unser ros-pkg/vision/calib_converter to convert 3d point
cloud from .bag file to OpenCV .xml file of CvMat. For example,

./calib_converter 08-08-01_calib_example/*.bag



RUNING THE TOOL:

to run the stereo and lrf calibration tool, do the following
.bin/stlrf

to load a sequence of images.
on the righthand side, push the button "load sequence", which brings up
a file loading dialog. Navigate to your data directory, select the first
left image (e.g. 000-L.png)  in the sequence, and click the "ok" button.

Try the following data set as an example

/wg/stor2/prdata/stereo_calib/08-08-08_calib

to load a triplet of images.
select the tab you want to load the image into, click the load all 3 button on
the right. In the file loading dialog that pops up, choose the left image of 
the triplet you want to load.

to run the calibration process, click the "Calibrate" button in lower center.
The rotation and shift matrices shall be printed on screen, followed by some 
error measurement messages.


CAVEAT:
Currently, the camera parameters are hard-wired in the code. 
Shall read from a file, or a ROS message.
The image plane parameters (like camera parameters) for synthesizing an image from 3d point clouds
are also hard-wired. 


///
///
/// Calibration of the stereo camera (videre)
///
///


For calibration of the stereo camera (videre)

bin/stereocal



