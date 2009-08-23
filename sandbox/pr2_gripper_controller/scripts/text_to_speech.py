import roslib
roslib.load_manifest('pr2_gripper_controller')
from std_msgs.msg import String
import rospy
import os
def speechRequestCallback(data):
  print data
  os.system("espeak -ven+f3 \"%s\"" %data.data)
rospy.Subscriber("text_to_speech", String, speechRequestCallback)
rospy.init_node('text_to_speech')
while not rospy.is_shutdown():
  rospy.spin()
