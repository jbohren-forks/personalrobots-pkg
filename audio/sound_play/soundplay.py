#!/usr/bin/env python
import roslib; roslib.load_manifest('sound_play')

import rospy
import threading
from sound_play.msg import SoundRequest
import pygame.mixer as mixer

import os

class soundtype:
    STOPPED = 0
    LOOPING = 1
    COUNTING = 2

    def __init__(self, file):
        self.lock = threading.Lock()
        self.state = self.STOPPED
        self.chan = None
        self.sound = mixer.Sound(file)

    def loop(self):  
        self.lock.acquire()
        try:
            if self.state == self.COUNTING:
                stop()
            
            if self.state == self.STOPPED:
                self.chan = self.sound.play(-1)
            
            self.state = self.LOOPING
        finally:
            self.lock.release()

    def stop(self):
        if self.state != self.STOPPED:
            self.lock.acquire()
            try:
                self.chan.fadeout(300)
                self.state = self.STOPPED
            finally:
                self.lock.release()

    def single(self):
        self.lock.acquire()
        try:
            if self.state == self.LOOPING:
                stop()
            
            if self.state == self.STOPPED:
                self.chan = self.sound.play()
            else: # Already counting
                self.chan.queue(self.sound) # This will only allow one extra one to be enqueued

            self.state = self.COUNTING
        finally:
            self.lock.release()

    def command(self, cmd):
         if cmd == SoundRequest.PLAY_STOP:
             self.stop()
         elif cmd == SoundRequest.PLAY_ONCE:
             self.single()
         elif cmd == SoundRequest.PLAY_START:
             self.loop()

class soundplay:
    def stopall(self):
        for sound in self.soundlist.values():
            sound.stop()

    def callback(self,data):
        self.mutex.acquire()
        
        try:
            if data.sound == SoundRequest.ALL and data.command == SoundRequest.PLAY_STOP:
                self.stopall()
            elif data.sound > 0:
                self.soundlist[data.sound].command(data.command)
        finally:
            self.mutex.release()

    # subscribe to velocity command (cmd_vel)
    # to see if the forward velocity (vel.vw) is negative

    def __init__(self):
        rospy.init_node('soundplay')

        rootdir = os.path.join(os.path.dirname(__file__),'sounds')
        
        self.mutex = threading.Lock()
        mixer.init(11025, -16, 1, 4000)
        self.soundlist = {
                SoundRequest.BACKINGUP              : soundtype(os.path.join(rootdir, 'BACKINGUP.ogg')),
                SoundRequest.NEEDS_UNPLUGGING       : soundtype(os.path.join(rootdir, 'NEEDS_UNPLUGGING.ogg')),
                SoundRequest.NEEDS_PLUGGING         : soundtype(os.path.join(rootdir, 'NEEDS_PLUGGING.ogg')),
                SoundRequest.NEEDS_UNPLUGGING_BADLY : soundtype(os.path.join(rootdir, 'NEEDS_UNPLUGGING_BADLY.ogg')),
                SoundRequest.NEEDS_PLUGGING_BADLY   : soundtype(os.path.join(rootdir, 'NEEDS_PLUGGING_BADLY.ogg')),
                }
        
        rospy.Subscriber("robotsound", SoundRequest, self.callback)

        rospy.spin()


if __name__ == '__main__':
    soundplay()

