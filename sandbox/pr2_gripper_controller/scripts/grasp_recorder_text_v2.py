import roslib
roslib.load_manifest('pr2_gripper_controller')
import rospy
import rosrecord

from pr2_msgs.msg import GripperControllerState
from ethercat_hardware.msg import PressureState
    
bag_file_list = open("bag_list_v2.txt", "r")
current_file = bag_file_list.readline()

delta_time = rospy.Duration(1.0/25.0)
starting_sum0 = 0
starting_sum1 = 0

while (current_file != ""):
  seen_topic1 = 0
  seen_topic2 = 0
  starting_sum0 = 0
  starting_sum1 = 0
  seen_20 = False
  seen_100_after_20 = False
  max_position = 0.0
  current_file = current_file.strip()
  text_file = open("%s.txt" %current_file, "w")
  print "opening %s.txt" %current_file
  #text_file.write("#position, velocity, force_finger0, force_finger1, time, min_finger0, max_finger0, min_finger1, max_finger1, mean_finger0, mean_finger1, mean_fingers, force_fingers, min_fingers, max_fingers\n")

  closing_gripper = 2 #0 = contacted, 1 = closing, 2 = opening
  
  for topic, data, t in rosrecord.logplayer(current_file):
    if (topic == "/pressure/r_gripper_motor"):
      seen_topic1 = 1
      data0 = data.data0
      data1 = data.data1
      if starting_sum0 == 0 and starting_sum1 == 0:
        last_time = t
    elif (topic == "/pr2_gripper_controller/state"):
      seen_topic2 = 1
      actual_velocity = data.joint_velocity
      actual_position = data.joint_position
      commanded_effort = data.joint_commanded_effort
    
    if (seen_topic1 + seen_topic2 == 2 and t > last_time + delta_time):
      #print "t %f, last %f, del %f" %(t.to_seconds(), last_time.to_seconds(), delta_time.to_seconds())
      last_time = t
      current_sum0 = 0
      current_sum1 = 0
      if (closing_gripper == 2) and actual_velocity < -0.001:
        closing_gripper = 1
        starting_sum0 = 0
        starting_sum1 = 0
        for j in range(7, 22):
          starting_sum0 = starting_sum0 + data0[j]
          starting_sum1 = starting_sum1 + data1[j]
        print "closing %f" %t.to_seconds()
        #start time was here
      elif closing_gripper == 1 and (commanded_effort == -20.0 or (commanded_effort == 100.0 and seen_20)):
        if(commanded_effort == -20.0 and not seen_20):
          seen_20 = True
          start_time = t
        if(commanded_effort == 100):
          seen_100_after_20 = True
        if(seen_20 and not seen_100_after_20 and max_position < actual_position):
          max_position = actual_position
        if(seen_20 and not seen_100_after_20):
          text_file_fingertip = open("%s-fingertip.txt" %current_file, "w")
          for j in range(7, 22):
            text_file_fingertip.write("%i, " %data0[j])
          text_file_fingertip.write("\n")
          for j in range(7, 22):
            text_file_fingertip.write("%i, " %data1[j])
          text_file_fingertip.close()
        if(seen_20 and seen_100_after_20 and max_position < actual_position):
          break
        min0 = 99999999
        max0 = 0
        min1 = 99999999
        max1 = 0
        current_sum0 = 0
        current_sum1 = 0
        for i in range(7, 22):
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
        text_file.write("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n" %(actual_position, actual_velocity, current_sum0-starting_sum0, current_sum1-starting_sum1, t.to_seconds()-start_time.to_seconds(), min0, max0, min1, max1,mean0, mean1, (mean0+mean1)/2,current_sum0+current_sum1-starting_sum0-starting_sum1,min,max,commanded_effort)) 
          
  text_file.close()
  print "done %f" %t.to_seconds()
  
  #if not successful:
    #print "!!!!!!!!!!!!!failed %s!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1" %current_file
  current_file = bag_file_list.readline()
