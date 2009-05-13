#!/usr/bin/env python
import roslib; roslib.load_manifest('backup_safetysound')

import rospy
import threading
from robot_msgs.msg import PoseDot
from sound_play.msg import SoundRequest

import os


# if robot base receives a command to move backwards in the x-axis,
# then play sound file (TruckBackUp.wav) from Logitech USB speaker (hw:1,0).
# data is an instance of a PoseDot extracted from the cmd_vel topic.
class backingup:
    def start(self):
        msg = SoundRequest()
        msg.sound = SoundRequest.BACKINGUP
        msg.command = SoundRequest.PLAY_START
        self.pub.publish(msg)
        self.targettime = rospy.get_time() + 1

    def stop(self):
        msg = SoundRequest()
        msg.sound = SoundRequest.BACKINGUP
        msg.command = SoundRequest.PLAY_STOP
        self.pub.publish(msg)
        self.state = self.STOPPED

    def callback(self,data):
        self.mutex.acquire()
        
        try:
            if data.vel.vx < 0:
                if self.state == self.STOPPED:
                    self.start()
                self.state = self.PLAYING
    
            if data.vel.vx >= 0 and self.state == self.PLAYING:
                self.state = self.STOPPING
                self.targettime = rospy.get_time() + 0.5

        finally:
            self.mutex.release()

    # subscribe to velocity command (cmd_vel)
    # to see if the forward velocity (vel.vw) is negative

    STOPPED = 0; # The sound is not playing
    PLAYING = 1; # The sound is playing, self.targettime says when to repeat the sound command.
    STOPPING = 2; # The sound should be stopped after self.targettime is reached.

    def __init__(self):
        rospy.init_node('backingup_safetysound')
        
        self.mutex = threading.Lock()
        self.state = self.STOPPED
        self.targettime = 0
        self.pub = rospy.Publisher('robotsound', SoundRequest)
        rospy.Subscriber("cmd_vel", PoseDot, self.callback)
    
        while not rospy.is_shutdown():
            rospy.sleep(.5)
            self.mutex.acquire()
            try:
                if self.state == self.STOPPING and self.targettime < rospy.get_time():
                    self.stop()
                
                if self.state == self.PLAYING and self.targettime < rospy.get_time():
                    self.start()
            finally:
                self.mutex.release()


if __name__ == '__main__':
    backingup()

