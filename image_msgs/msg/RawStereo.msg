# This message defines all the information necessary to reconstruct a
# full set of stereo information.  It should be generated directly by
# a driver connected to a set of stereo cameras.  It is only intended
# to be fed into a stereo processing node, and should not otherwise be
# used.  This is the preferred message to log when generating log
# files, as it is the minimal representation of the information.

byte       NONE=0
byte       IMAGE_RAW=1
byte       IMAGE=2
byte       IMAGE_COLOR=3
byte       IMAGE_RECT=4
byte       IMAGE_RECT_COLOR=5

Header     header
StereoInfo stereo_info
CamInfo    left_info
byte       left_type
Image      left_image
CamInfo    right_info
byte       right_type
Image      right_image
Image      disparity_image
