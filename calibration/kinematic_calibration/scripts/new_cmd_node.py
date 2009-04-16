#!/usr/bin/env python
import roslib
roslib.load_manifest('kinematic_calibration')

import time
import rospy
import sys
from std_msgs.msg import Empty
from pr2_mechanism_controllers.srv import *
from pr2_mechanism_controllers.msg import *
from robot_msgs.msg import *
from image_msgs.msg import *

from kinematic_calibration.arm_commander import ArmCommander
from kinematic_calibration.settler import *
from kinematic_calibration.msg_cache import MsgCache
from kinematic_calibration.image_point_stream import *

from roslib import rostime

rospy.init_node('arm_commander', sys.argv, anonymous=False)

pub = rospy.Publisher('~cal_snapshot', CalSnapshot)

# Initialize Arm Command Stuff
arm_commander = ArmCommander('right_arm_trajectory_controller')
settler = Settler(100)
settler.start('mechanism_state')

# Initialize imagePoint stuff

cam_configs = [ ]
cam_configs.append( {'topic' :'/stereo_left_detector/led',
                     'sensor':'stereo_left',
                     'target':'gripper_right',
                     'skew'  : .25, 'tol': 1,
                     'cache_size' : 40,
                     'min_samples': 3 } )

cam_configs.append( {'topic' :'/stereo_right_detector/led',
                     'sensor':'stereo_right',
                     'target':'gripper_right',
                     'skew'  : .25, 'tol': 1,
                     'cache_size' : 40,
                     'min_samples': 3 } )

cams = [ImagePointStream(x) for x in cam_configs]

map(lambda x: x.start(), cams)

joints = arm_commander.get_traj_joint_names()
for k in range(0,40) :

    # Command arm into the correct location
    print "Command traj %u" % k
    result = arm_commander.cmd_arm(k)
    print "   Result: %u" % result

    led_settled = False
    arm_settled = False
    repeat_count = 0
    while not (arm_settled and led_settled) and repeat_count < 3 and not rospy.is_shutdown() :
        print "   Waiting for arm to settle..."
        arm_settled = False
        while not arm_settled and not rospy.is_shutdown() :
            arm_stats = settler.get_stats_latest(joints, 100)
            arm_settled = True
            for x in arm_stats.ranges :
                arm_settled = arm_settled and (x < .0001)
            time.sleep(.1)
        print "   Arm is settled!"        

        # Grab a stereotypical MechanismState that's close to the middle of the interval
        cur_mech_state = arm_stats.seg[len(arm_stats.seg)/2]

        # Grab LED information from this interval
        led_stats  = map( (lambda x:x.get_stats_interval(arm_stats.start, arm_stats.end)), cams )
        led_stable = map( (lambda x,y:x.is_stable(y)), cams, led_stats )
        samples = [ ]
        for k in range(0,len(cams)) :
            if led_stable[k] :
                samples.append(cams[k].build_sample(led_stats[k]))

        repeat_count = repeat_count + 1
        if (len(samples) > 0) :
            print "Found LED!"
            led_settled = True
            msg = CalSnapshot()
            msg.mech_state = cur_mech_state
            msg.samples = samples
            print "Publishing"
            pub.publish(msg)
            print "    [" + ', '.join( ['%.3f'%x for x in arm_stats.ranges] ) + "]"
            for s in msg.samples :
                print "    [%3.2f, %3.2f]" % (s.m[0], s.m[1])
            
        else :
            print "   Found no leds...sleeping...",
            sys.stdout.flush()
            led_settled = False
            time.sleep(1)
            print "done"

