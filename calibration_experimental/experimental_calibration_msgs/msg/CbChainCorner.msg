# Stores a single checkerboard corner, as sensed by some kinematic chain
Header header
int16 x_index            # The x index of the checkerboard corner
int16 y_index            # The y index of the checkerboard corner
JointPosArray joints   # The system's joint positions when sensing this corner