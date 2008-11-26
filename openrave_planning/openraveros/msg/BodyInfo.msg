# information about a body
uint32 bodyid

# filename used to create body geometry
string filename

# name of the body
string name

# type of body
string type

# current transformation
AffineTransformMatrix transform

# degrees of freedom (number of joints)
byte dof

# enabled status
byte enabled

float32[] jointvalues
AffineTransformMatrix[] links
string[] linknames

# request information bitmasks, also holds robot specific request information
uint16 Req_Transform=1
uint16 Req_JointValues=2
uint16 Req_Links=4
uint16 Req_LinkNames=8
uint16 Req_JointLimits=16
uint16 Req_RobotManipulators=32
uint16 Req_RobotSensors=64
uint16 Req_RobotActiveDOFs=128
