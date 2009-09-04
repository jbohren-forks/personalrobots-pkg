uint32 num_x     # Number of checkerboard corners in the X direction
uint32 num_y     # Number of corners in the Y direction
float32 spacing_x  # Spacing between corners in the X direction (meters)
float32 spacing_y  # Spacing between corners in the Y direction (meters)

# Specify how many times we want to upsample the image.
#  This is often useful for detecting small checkerboards far away
float32 width_scaling
float32 height_scaling

# Specifiy how intensity maps into a uint8. A specified window of
#   intensities is linearly scaled to 0-255
float32 min_intensity
float32 max_intensity

uint32 subpixel_window
int32  subpixel_zero_zone

