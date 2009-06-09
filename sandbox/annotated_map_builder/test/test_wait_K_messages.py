#!/usr/bin/env python


import roslib
roslib.load_manifest('annotated_map_builder')
import rospy
import random
from image_msgs.msg import RawStereo

from annotated_map_builder.wait_for_k_messages_adapter import *

if __name__=='__main__':
    rospy.init_node("ExecutiveDataCollector", anonymous=True)

    capture_waiter = WaitForKMessagesAdapter("/stereo/raw_stereo",RawStereo,50,10)
    capture_waiter.startWaiting();

    for i in range(0,100):
        if rospy.is_shutdown():
            break

        print i,"done?",capture_waiter.doneWaiting(),capture_waiter.msg_wait_count_down_
        sleep_time = 1.0
        rospy.sleep(sleep_time)
