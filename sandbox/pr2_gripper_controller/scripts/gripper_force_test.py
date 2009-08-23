import roslib
roslib.load_manifest('pr2_gripper_controller')
from pr2_mechanism_controllers.msg import GripperControllerCmd
from pr2_msgs.msg import GripperControllerState
from ethercat_hardware.msg import PressureState
from std_msgs.msg import Float64
import rospy
import time

open_between = 1
voltage = 10.0 #volts
load_cell_max = 50 #lbs
load_cell_mV_per_V = 2 #mV/V
vel_threshold = .001
text_file = open("pressure_values.txt", "w")
def gripper_state_callback(data):
  global actual_effort
  global actual_velocity
  actual_effort = data.joint_applied_effort
  actual_velocity = data.joint_velocity
  
def pressure_callback(data):
  global data0
  global data1
  data0 = data.data0
  data1 = data.data1
  
def load_cell_callback(data):
  global load_cell_V
  load_cell_V = data.data
  
pub = rospy.Publisher('r_gripper_cmd', GripperControllerCmd)
rospy.Subscriber("agilent_measure", Float64, load_cell_callback)
rospy.Subscriber("r_gripper/state", GripperControllerState, gripper_state_callback)
rospy.Subscriber("pressure/r_gripper_motor", PressureState, pressure_callback)
  
rospy.init_node('gripper_force_test')
cmd = GripperControllerCmd()
cmd.cmd = 'open'
cmd.start = 0.0
cmd.end = 0.0
cmd.time = 0.0
cmd.val = 0.0
pub.publish(cmd)
time.sleep(3.0)
starting_offset_voltage = load_cell_V
starting_sum0 = 0
starting_sum1 = 0

for j in range(7, 21):
  starting_sum0 = starting_sum0 + data0[j]
  starting_sum1 = starting_sum1 + data1[j]

for i in range (13, 130):
  print "i is %i", i
  ending_sum0 = 0
  ending_sum1 = 0
  if open_between == 1:
    starting_sum0 = 0
    starting_sum1 = 0
    cmd.cmd = 'open'
    cmd.val = 2.0
    print "opening"
    pub.publish(cmd)
    time.sleep(.1)
    while not (-vel_threshold < actual_velocity < vel_threshold):
      time.sleep(.25)
    for j in range(7, 21):
      starting_sum0 = starting_sum0 + data0[j]
      starting_sum1 = starting_sum1 + data1[j]
  cmd.cmd = 'move'
  cmd.val = i*-1
  print "moving, cmd val is %f", cmd.val
  pub.publish(cmd)
  if open_between == 1:
    time.sleep(.25)
    while not (-vel_threshold < actual_velocity < vel_threshold):
      time.sleep(.25)
  time.sleep(1)
  for j in range(7, 21):
    ending_sum0 = ending_sum0 + data0[j]
    ending_sum1 = ending_sum1 + data1[j]
  print "force is %f", (load_cell_V * 1000.0 - starting_offset_voltage) * load_cell_max / (load_cell_mV_per_V * voltage)
  text_file.write("%f %f %f %f %f %f\n" %(cmd.val, actual_effort, ending_sum0-starting_sum0, ending_sum1-starting_sum1, load_cell_V, (load_cell_V * 1000.0 - starting_offset_voltage) * load_cell_max / (load_cell_mV_per_V * voltage)));
text_file.close()
cmd.cmd = 'open'
pub.publish(cmd)