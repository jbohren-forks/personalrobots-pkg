import roslib
roslib.load_manifest('pr2_gripper_controller')
import rospy
import time
import os
import threading

from pr2_msgs.msg import GripperControllerState
from ethercat_hardware.msg import PressureState

def pressure_callback(data):
  global data0
  global data1
  data0 = data.data0
  data1 = data.data1
  
def gripper_state_callback(data):
  global actual_velocity
  global actual_position
  actual_velocity = data.joint_velocity
  actual_position = data.joint_position
  
rospy.Subscriber("pressure/r_gripper_motor", PressureState, pressure_callback)
rospy.Subscriber("pr2_gripper_controller/state", GripperControllerState, gripper_state_callback)

rospy.init_node('grasp_recorder')

class bagPlayer (threading.Thread):
  def run (self):
    os.system("rosplay %s" %current_file)
    print "SPAM"

bag_file_list = open("bag_list2.txt", "r")
current_file = bag_file_list.readline()
while (current_file != ""):
  current_file = current_file.strip()

  playerThread = bagPlayer()
  playerThread.start()
  
  rospy.sleep(2)

  text_file = open("%s.txt" %current_file, "w")
  print "opening %s.txt" %current_file
  text_file.write("#position, velocity, force_finger0, force_finger1, time, min_finger0, max_finger0, min_finger1, max_finger1, mean_finger0, mean_finger1, mean_fingers, force_fingers, min_fingers, max_fingers\n")
  
  starting_sum0 = 0
  starting_sum1 = 0
  
  for j in range(7, 21):
    starting_sum0 = starting_sum0 + data0[j]
    starting_sum1 = starting_sum1 + data1[j]
  print "closing %f" %rospy.get_time()
  contact = 0
  while 1:
    current_sum0 = 0
    current_sum1 = 0
    for i in range(7, 21):
      current_sum0 = current_sum0 + data0[i]
      current_sum1 = current_sum1 + data1[i]
    if current_sum0 > starting_sum0 + 1000 and current_sum1 > starting_sum1 + 1000:
      break
    
  start_time = rospy.get_time()
  print "closed %f" %rospy.get_time()
  
  while actual_velocity < -0.001:
    min0 = 99999999
    max0 = 0
    min1 = 99999999
    max1 = 0
    current_sum0 = 0
    current_sum1 = 0
    for i in range(7, 21):
      current_sum0 = current_sum0 + data0[i]
      current_sum1 = current_sum1 + data1[i]
      if data0[i] > max0:
        max0 = data0[i]
      if data0[i] < min0:
        min0 = data0[i]
      if data1[i] > max1:
        max1 = data1[i]
      if data1[i] < min1:
        min1 = data1[i]
    mean0 = current_sum0/15.0-starting_sum0/15
    mean1 = current_sum1/15.0-starting_sum1/15
    min = 0
    max = 0
    if min0 < min1:
      min = min0
    else:
      min = min1
    if max0 > max1:
      max = max0
    else:
      max = max1
    text_file.write("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n" %(actual_position, actual_velocity, current_sum0-starting_sum0, current_sum1-starting_sum1, rospy.get_time()-start_time, min0, max0, min1, max1,mean0, mean1, (mean0+mean1)/2,current_sum0+current_sum1-starting_sum0-starting_sum1,min,max)) 
    rospy.sleep(1.0/25.0)
  text_file.close()
  print "done %f" %rospy.get_time()
  playerThread.join()
  current_file = bag_file_list.readline()
