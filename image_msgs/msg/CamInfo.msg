# This message defines meta information for a camera. It should be in a
# camera namespace and accompanied by up to 5 image topics named:
# 
# image_raw, image, image_color, image_rect, and image_rect_color
# 
# Whether these other topics are being published is defined by
# the has_* variables within this message.

Header header

uint32 height
uint32 width

float64[5]  D # Distortion: k1, k2, t1, t2, t3
float64[9]  K # original camera matrix
float64[9]  R # rectification matrix
float64[12] P # projection/camera matrix

byte has_image_raw
byte has_image
byte has_image_color
byte has_image_rect
byte has_image_rect_color

# Should put exposure, gain, etc. information here as well