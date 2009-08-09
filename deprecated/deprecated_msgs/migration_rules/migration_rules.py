class update_robot_msgs_VOPose_0b686775e7d79f53073ac33f28f73022(MessageUpdateRule):
	old_type = "robot_msgs/VOPose"
	old_full_text = """
Header header
robot_msgs/Pose pose
int32 inliers

================================================================================
MSG: roslib/Header
#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: robot_msgs/Pose
Point position
Quaternion orientation

================================================================================
MSG: robot_msgs/Point
float64 x
float64 y
float64 z

================================================================================
MSG: robot_msgs/Quaternion
# xyz - vector rotation axis, w - scalar term (cos(ang/2))
float64 x
float64 y
float64 z
float64 w
"""

	new_type = "deprecated_msgs/VOPose"
	new_full_text = """
Header header
geometry_msgs/Pose pose
int32 inliers

================================================================================
MSG: roslib/Header
#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Pose
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# xyz - vector rotation axis, w - scalar term (cos(ang/2))
float64 x
float64 y
float64 z
float64 w
"""

	order = 0
	migrated_types = [
		("Header","Header"),
                ("Pose", "geometry_msgs/Pose")]

	valid = True

	def update(self, old_msg, new_msg):
		self.migrate(old_msg.header, new_msg.header)
                self.migrate(old_msg.pose, new_msg.pose)
		new_msg.inliers = old_msg.inliers

