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
float64[12] RP # Reprojection Matrix

int32 dpp
int32 numDisp
int32 imDtop
int32 imDleft
int32 imDwidth
int32 imDheight
int32 corrSize
int32 filterSize
int32 horOffset
int32 textureThresh
int32 uniqueThresh

byte has_disparity

# Should put exposure, gain, etc. information here as well