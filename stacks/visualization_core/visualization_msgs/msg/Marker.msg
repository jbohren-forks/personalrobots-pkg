byte ARROW=0
byte CUBE=1
byte SPHERE=2
byte CYLINDER=3
byte LINE_STRIP=4
byte LINE_LIST=5
byte CUBE_LIST=6
byte SPHERE_LIST=7
byte POINTS=8

byte ADD=0
byte MODIFY=0
byte DELETE=2

Header header                        # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 type 		                       # Type of object
int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object
robot_msgs/Pose pose                 # Pose of the object
robot_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color             # Color [0.0-1.0]
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever

#Only used if the type specified has some use for them (eg. LINE_STRIP)
robot_msgs/Point[] points
