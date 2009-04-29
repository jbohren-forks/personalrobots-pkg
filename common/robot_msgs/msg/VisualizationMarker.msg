#
# Visualization Marker message
# For inserting pre-made objects into the visualization
#

byte ARROW=0
byte CUBE=1
byte SPHERE=2
byte ROBOT=3
byte CYLINDER=4
byte LINE_STRIP=5
byte LINE_LIST=6

byte ADD=0
byte MODIFY=0
byte DELETE=2

Header header # header for time/frame information
int32 id 		# object ID useful for manipulating and deleting the object later
int32 type 		# Type of object
int32 action 	# 0 add/modify an object, 1 (deprecated), 2 deletes an object
float32 x 		# x location of the object
float32 y 		# y location of the object
float32 z 		# z location of the object
float32 roll 	# roll of the object
float32 pitch 	# pitch of the object
float32 yaw 	# yaw of the object
float32 xScale 	# scale of the object in the x direction (1 does not change size) (default sizes are usually 1 meter square)
float32 yScale 	# scale in y direction
float32 zScale 	# scale in z direction
int32 alpha 	# alpha (0-255) (amount of transparency, 255 is opaque)
int32 r 		# red color value (0-255) (if available)
int32 g 		# green color value
int32 b 		# blue color value
#These are only used if the object type is LINE_STRIP or LINE_LIST
Point[] points
