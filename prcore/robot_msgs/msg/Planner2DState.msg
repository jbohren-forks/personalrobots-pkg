Header header


#Status options
byte INACTIVE=0
byte ACTIVE=1
byte ABORTED=2
byte PREEMPTED=3

# Status of the controller = {INACTIVE, ACTIVE, ABORTED, PREEMPTED}
byte status

#Id of the goal
int32 goal_id

#Comment for debug
string comment

# Did the planner find a valid path?
byte valid
# Have we arrived at the goal?
byte done
# Current location (m,m,rad)
deprecated_msgs/Pose2DFloat32 pos
# Goal location (m,m,rad)
deprecated_msgs/Pose2DFloat32 goal
# Current waypoint location (m,m,rad)
deprecated_msgs/Pose2DFloat32 waypoint
# Current list of waypoints in the plan
deprecated_msgs/Pose2DFloat32[] waypoints
# Current waypoint index. May be negative if there's no plan, or if
# the plan is done
int32 waypoint_idx
