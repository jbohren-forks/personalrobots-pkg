#!/usr/bin/env python
import roslib; roslib.load_manifest('sound_play')

import rospy
from sound_play.msg import SoundRequest

def play(sound):
        msg = SoundRequest()
        msg.sound = sound
        msg.command = SoundRequest.PLAY_ONCE
        pub.publish(msg)

def start(sound):
        msg = SoundRequest()
        msg.sound = sound
        msg.command = SoundRequest.PLAY_START
        pub.publish(msg)

def stop(sound):
        msg = SoundRequest()
        msg.sound = sound
        msg.command = SoundRequest.PLAY_STOP
        pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('soundplay_test', anonymous = True)
    pub = rospy.Publisher('robotsound', SoundRequest)

    rospy.sleep(1)
    
    stop(SoundRequest.ALL)

    while not rospy.is_shutdown():
        print 'plugging'
        play(SoundRequest.NEEDS_PLUGGING)
        play(SoundRequest.NEEDS_PLUGGING)

        rospy.sleep(4)

        #start(SoundRequest.BACKINGUP)

        rospy.sleep(1)

        print 'unplugging'
        play(SoundRequest.NEEDS_UNPLUGGING)

        rospy.sleep(1)
        print 'plugging badly'
        play(SoundRequest.NEEDS_PLUGGING_BADLY)
        rospy.sleep(1)
        #stop(SoundRequest.BACKINGUP)

        rospy.sleep(2)
        print 'unplugging badly'
        play(SoundRequest.NEEDS_UNPLUGGING_BADLY)

        rospy.sleep(5)
        
