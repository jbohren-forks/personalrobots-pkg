# information specific to a manipulator for a robot

# zero-based index of base link that manipulator is attached to
byte baselink

# zero-based index of link defining the end-effector
byte eelink

# 3x4 matrix of the grasp frame relative to the end-effector link
AffineTransformMatrix tgrasp

# zero-based hand joint indices
byte[] handjoints

# zero-based arm joints indices (from base to end-effector)
byte[] armjoints

# name of ik solver using
string iksolvername
