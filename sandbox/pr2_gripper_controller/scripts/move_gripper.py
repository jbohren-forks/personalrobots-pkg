import roslib; roslib.load_manifest('pr2_gripper_controller')
from pr2_mechanism_controllers.msg import GripperControllerCmd
import rospy
import time

rospy.init_node('gripper_controller_test')
cmd = GripperControllerCmd()
cmd.cmd = 'move'
cmd.start = 0
cmd.end = 0
cmd.time = 0

cmd.val = 2
pub = rospy.Publisher('r_gripper_cmd', GripperControllerCmd)

force = -0.5
while not rospy.is_shutdown():
  force = force - 0.25
  cmd.val = force
  pub.publish(cmd)
  time.sleep(3)
  cmd.val = 20 
  pub.publish(cmd)
  time.sleep(3) 
