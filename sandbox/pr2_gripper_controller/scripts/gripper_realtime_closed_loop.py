from Queue import Full
import roslib
roslib.load_manifest('pr2_gripper_controller')
from std_msgs.msg import String
import rospy
import time
import os
#import curses
from experimental_controllers.srv import *
#stdscr = curses.initscr()
rospy.init_node('grasp_recognition_client')
pub = rospy.Publisher('/text_to_speech', String)
print "waiting"
rospy.wait_for_service('grasp_closed_loop')
print "waited"
try:
  cl_grasp_req = GraspClosedLoopRequest()
  cl_grasp_req.cmd = "grasp"
  cl_grasp_req.trials = 3
  print "getting reference" 
  cl_grasp = rospy.ServiceProxy('grasp_closed_loop',GraspClosedLoop)
  print "asking for response"
  response = cl_grasp.call(cl_grasp_req)
  #print "%s %s\nresult: %s\ntrials: %s\ndist: %s\neff: %s\npeak0: %s\npeak1: %s\nsteady0: %s\nsteady1: %s\ncompressed0: %s\npeak: %s\nstiffness: %s" %(object_type, object_state,cl_grasp_req.trials,response.result, response.distance, response.effort, response.force_peak0, response.force_peak1, response.force_steady0, response.force_steady1, response.distance_compressed, response.distance_peak, response.stiffness)
  #f.write('%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' %(object_type, object_state, response.result, response.velocity, response.effort, response.distance, response.force_peak0, response.force_peak1, response.force_steady0, response.force_steady1, response.distance_compressed, response.distance_peak))
  obj = 'odwalla'
  obj_amount = ''
  obj_state = ''
  if(response.time[2] <= 0.542):
    if response.force_peak0[2] <= 14609:
      obj_amount = 'empty'
      if response.distance_compressed[2] <= 0.049578:
        obj_state = 'open'
      else:
        if response.force_steady0[2] <= 9386:
          obj_state = 'open'
        else:
          obj_state = 'closed'
    else:
      obj_amount = 'full'
      obj_state = 'closed'
  else:
    obj_amount = 'full'
    obj_state = 'open'
  result_output = "The bottle is %s and %s" %(obj_amount,obj_state)
  print result_output
  pub.publish(result_output)
except rospy.ServiceException, e:
  print "Service call failed: %s"%e

