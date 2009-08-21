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
text_file = open("stiction_points.txt", "r")
text_file_2 = open("stiction_points_verified.txt", "w")
time.sleep(.5)
pub.publish(cmd)
line = text_file.readline()
num = 1
while not rospy.is_shutdown() and line:
  print "On num %i, moving to %s"%(num,line.split(' ')[1])
  cmd.cmd = 'moveTo'
  cmd.val = float(line.split(' ')[1])
  pub.publish(cmd)
  time.sleep(.5)
  while not (-vel_threshold < actual_velocity < vel_threshold):
    time.sleep(.25)
  cmd.cmd = 'move'
  cmd.val = float(line.split(' ')[2])
  pub.publish(cmd)
  rospy.sleep(.2)
  if vel_threshold < actual_velocity:
    text_file_2.write('1\n')
    print '1'
  elif actual_velocity < -1.0*vel_threshold:
    text_file_2.write('-1\n')
    print '-1'
  else:
    text_file_2.write('0\n')
    print '0'
  line = text_file.readline()
  num = num + 1
text_file_2.close()
