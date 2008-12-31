# This message defines meta information for a stereo pair. It should
# be in a stereo namespace and accompanied by 2 camera namespaces, and
# a disparity image, named:
#
# left, right, and disparity, respectively

Header header

uint32 height
uint32 width

float64[3]  T  # Pose of right camera in left camera coords
float64[3]  Om # rotation vector
float64[16] RP # Reprojection Matrix

int32 dpp
int32 num_disp
int32 im_Dtop
int32 im_Dleft
int32 im_Dwidth
int32 im_Dheight
int32 corr_size
int32 filter_size
int32 hor_offset
int32 texture_thresh
int32 unique_thresh
int32 smooth_thresh
int32 speckle_diff
int32 speckle_region_size
byte  unique_check

byte has_disparity

# Should put exposure, gain, etc. information here as well