# This message contains information about limits of a particular joint (or control dimension)

uint8 has_position_limits
uint8 angle_wraparound
float64 min_position
float64 max_position

uint8 has_velocity_limits
float64 max_velocity
# min_velocity is assumed to be -max_velocity
