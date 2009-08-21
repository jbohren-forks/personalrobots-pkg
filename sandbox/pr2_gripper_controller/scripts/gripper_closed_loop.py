import roslib
roslib.load_manifest('pr2_gripper_controller')
import rospy
import time
#import curses
from experimental_controllers.srv import *
#stdscr = curses.initscr()
f = open('blah.txt', 'a')
object_type = "test"
object_state = "test,test,test"
print "waiting"
rospy.wait_for_service('grasp_closed_loop')
print "waited"
try:
  #curses.noecho()
  #print "waiting for user input: object type\n"
  #c1 = stdscr.getch() #object
  #print "waiting for user input: object state\n"
  #c2 = stdscr.getch() #open or closed/ big small
  #print "waiting for user input: object characteristic\n"
  #c3 = stdscr.getch() #full or empty/ plastic or real
  cl_grasp_req = GraspClosedLoopRequest()
  cl_grasp_req.cmd = "grasp"
  cl_grasp_req.trials = 3
  #cl_grasp_req.distance = 0.0584475
  #cl_grasp_req.distance_tolerance = 0.004
  #cl_grasp_req.effort = -20.0
  #cl_grasp_req.effort_tolerance = 5.0
  #cl_grasp_req.stiffness = 1.016
  #cl_grasp_req.stiffness_threshold = .002
  print "getting reference" 
  cl_grasp = rospy.ServiceProxy('grasp_closed_loop',GraspClosedLoop)
  print "asking for response"
  response = cl_grasp.call(cl_grasp_req)
  #print "%s %s\nresult: %s\ntrials: %s\ndist: %s\neff: %s\npeak0: %s\npeak1: %s\nsteady0: %s\nsteady1: %s\ncompressed0: %s\npeak: %s\nstiffness: %s" %(object_type, object_state,cl_grasp_req.trials,response.result, response.distance, response.effort, response.force_peak0, response.force_peak1, response.force_steady0, response.force_steady1, response.distance_compressed, response.distance_peak, response.stiffness)
  #f.write('%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' %(object_type, object_state, response.result, response.velocity, response.effort, response.distance, response.force_peak0, response.force_peak1, response.force_steady0, response.force_steady1, response.distance_compressed, response.distance_peak))
  f.write('%s,%s,%s,%s,' %(object_type, object_state, response.result, response.velocity))
  print '%s,%s,%s,%s' %(object_type, object_state, response.result, response.velocity)
  for i in range (0,cl_grasp_req.trials):
    f.write ('%s,' %response.effort[i])
    print 'effort: %s,' %response.effort[i]
  for i in range (0,cl_grasp_req.trials):
    f.write ('%s,' %response.distance[i])
    print 'distance: %s,' %response.distance[i]
  for i in range (0,cl_grasp_req.trials):
    f.write ('%s,' %response.force_peak0[i])
    print 'peak force0: %s,' %response.force_peak0[i]
  for i in range (0,cl_grasp_req.trials):
    f.write ('%s,' %response.force_peak1[i])
    print 'peak force1 %s,' %response.force_peak1[i]
  for i in range (0,cl_grasp_req.trials):
    f.write ('%s,' %response.force_steady0[i])
    print 'steady force0 %s,' %response.force_steady0[i]
  for i in range (0,cl_grasp_req.trials):
    f.write ('%s,' %response.force_steady1[i])
    print 'steady force1 %s,' %response.force_steady1[i]
  for i in range (0,cl_grasp_req.trials):
    f.write ('%s,' %response.distance_compressed[i])
    print 'distance compressed %s,' %response.distance_compressed[i]
  for i in range (0,cl_grasp_req.trials):
    f.write ('%s,' %response.distance_peak[i])
    print 'distance peak %s,' %response.distance_peak[i]
  for i in range (0,cl_grasp_req.trials):
    f.write ('%s,' %response.stiffness[i])
    print 'stiffness %s,' %response.stiffness[i]
  for i in range (0,cl_grasp_req.trials):
    f.write ('%s,' %response.time[i])
    print 'time %s,' %response.time[i]
  for i in range (0,cl_grasp_req.trials - 1):
    f.write ('%s,' %response.time_to_first[i])
    print 'time to first %s,' %response.time_to_first[i]
  for i in range (0,cl_grasp_req.trials - 1):
    f.write ('%s,' %response.time_to_return[i])
    print 'time to return %s,' %response.time_to_return[i]
  f.write('\n')
except rospy.ServiceException, e:
  print "Service call failed: %s"%e
f.close()
