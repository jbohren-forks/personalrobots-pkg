import roslib
roslib.load_manifest('pr2_gripper_controller')
from pr2_mechanism_controllers.msg import GripperControllerCmd
from pr2_msgs.msg import GripperControllerState
import rospy
import time

vel_threshold = .001
actual_velocity = 0.0
def callback(data):
  global actual_velocity
  global actual_position
  actual_velocity = data.joint_velocity
  actual_position = data.joint_position

rospy.init_node('stiction_test')
cmd = GripperControllerCmd()
cmd.cmd = 'moveTo'
cmd.start = 0.0
cmd.end = 0.0
cmd.time = 0.0
cmd.val = 0.0
pub = rospy.Publisher('r_gripper_cmd', GripperControllerCmd)
rospy.Subscriber("r_gripper/state", GripperControllerState, callback)
text_file = open("stiction_points.txt", "w")
time.sleep(.5)
pub.publish(cmd)

for i in range (0, 82):
  cmd.cmd = 'moveTo'
  cmd.val = i*.001
  pub.publish(cmd)
  time.sleep(.25)
  print "moving to ", i
  while not (-vel_threshold < actual_velocity < vel_threshold):
    time.sleep(.05)
  cmd.cmd = 'move'
  current_position = actual_position
  print "ramping", i
  for j in range (1, 400):
    cmd.val = j*.1
    pub.publish(cmd)
    #time.sleep(.5)
    rospy.sleep(.1)
    print "i is %d, j is %d, actual vel is %f" %(i, j, actual_velocity)
    if vel_threshold < actual_velocity:
      text_file.write('%f %f %f \n' %(i*.001, current_position, j*.1))
      print "found %d at %d" %(i, j)
      break
for i in range (0, 82):
  cmd.cmd = 'moveTo'
  cmd.val = i*.001
  pub.publish(cmd)
  time.sleep(.25)
  print "moving to ", i
  while not (-vel_threshold < actual_velocity < vel_threshold):
    time.sleep(.05)
  cmd.cmd = 'move'
  current_position = actual_position
  print "ramping", i
  for j in range (1, 400):
    cmd.val = j*-.1
    pub.publish(cmd)
    #time.sleep(.5)
    rospy.sleep(.1)
    print "i is %d, j is %d, actual vel is %f" %(i, j, actual_velocity)
    if -vel_threshold > actual_velocity:
      text_file.write('%f %f %f \n' %(i*.001, current_position, j*-.1))
      print "found %d at %d" %(i, j)
      break
text_file.close()
