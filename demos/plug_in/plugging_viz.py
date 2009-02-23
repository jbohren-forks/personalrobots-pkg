#!/usr/bin/env python
import roslib; roslib.load_manifest('plug_in')

import rospy
from robot_msgs.msg import Transform
from std_msgs.msg import VisualizationMarker

def callback(data, pub):
    VisualizationMarker marker
    
    marker.header.frame_id = "torso_lift_link"
    marker.header.stamp = ros::Time((uint64_t)0ULL)
    marker.id = 0
    marker.type = VisualizationMarker.SPHERE
    marker.action = VisualizationMarker.ADD
    marker.x = data.translation.x
    marker.y = data.translation.y
    marker.z = data.translation.z
    marker.yaw = 0
    marker.pitch = 0
    marker.roll = 0.0
    marker.xScale = 0.1
    marker.yScale = 0.1
    marker.zScale = 0.1
    marker.alpha = 255
    marker.r = 0
    marker.g = 255
    marker.b = 0
    pub.publish(marker)


def listener():
    viz_pub = rospy.Publisher('visualizationMarker', VisualizationMarker)
    rospy.init_node('plugging_viz_listener', anonymous=True)
    rospy.Subscriber("arm_constraint/transform", Transform, callback, viz_pub)

    rospy.spin()

if __name__ == '__main__':
    listener()

