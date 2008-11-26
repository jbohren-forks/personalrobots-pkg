# information specific to an attached sensor for a robot

# name of sensor
string name

# zero-based index of link sensor is attached to
byte attachedlink

# 3x4 matrix of the relative transform of the sensor frame with respect to the link frame
AffineTransformMatrix trelative

# type of sensor
string type
