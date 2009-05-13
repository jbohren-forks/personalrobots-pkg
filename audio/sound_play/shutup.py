#!/usr/bin/env python
import roslib; roslib.load_manifest('sound_play')

import rospy
from sound_play.msg import SoundRequest

def stop(sound):
        msg = SoundRequest()
        msg.sound = sound
        msg.command = SoundRequest.PLAY_STOP
        pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('shutup', anonymous = True)
    pub = rospy.Publisher('robotsound', SoundRequest)

    while not rospy.is_shutdown():
        stop(SoundRequest.ALL)
        rospy.sleep(.1)
