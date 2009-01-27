#!PYTHONPATH=$PYTHONPATH:`rospack find matplotlib`/src /usr/bin/ipython -i 
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# Written by Timothy Hunter <tjhunter@willowgarage.com> 2008
from __future__ import division

import rostools

# Loads interface with the robot.
rostools.load_manifest('teleop_robot')
from teleop_robot import *

# Manually loads arm interface
rostools.load_manifest('pr2_controllers')
from pr2_controllers import *

# Import plotting/data utils
rostools.load_manifest('scipy')
from scipy import *

rostools.load_manifest('matplotlib')
from pylab import *

# Data collecting utils
rostools.load_manifest('wxpy_ros')
import wxpy_ros

def bodeplot(fi,f,tf,clear=True):
    figure(fi)
    if clear:
        clf()
    subplot(211)
    semilogx(f,20*log10(abs(tf)))
    ylabel('Mag. Ratio (dB)')
    subplot(212)
    semilogx(f,arctan2(imag(tf),real(tf))*180.0/pi)
    ylabel('Phase (deg.)')
    xlabel('Freq. (Hz)')


import time
import copy

#c = ArmPositionProxy('left_arm_controller')

ri=RobotInterface()
#ac = ri.spawnFile('pr2_arm/gazebo_blind_calibration_upper_arm.xml')
#ac = ri.spawnFile('pr2_arm/gazebo_blind_calibration_elbow.xml')
#ac.calibrate()



## Create a message handler
#messageHandler = wxpy_ros.getHandler()
## Unfortunately, we still need the #id of the element for now
#channelY = messageHandler.subscribe('/mechanism_state/joint_states[%i]/position'%pos, '')
#channelU = messageHandler.subscribe('/mechanism_state/joint_states[%i]/applied_effort'%pos, '')
#channelCU = messageHandler.subscribe('/mechanism_state/joint_states[%i]/applied_effort'%pos, '')
## Starts the sweep
#sw.sweep(fmin,fmax,t,1)
## Cleans the channel
#channelY.flush()
#channelU.flush()
#channelCU.flush()
## Wait the end of the sweep
#time.sleep(t-1)
## Copy the data
#x = copy.deepcopy(channelY.x)
#y = copy.deepcopy(channelY.y)
#u = copy.deepcopy(channelU.y)
#cu = copy.deepcopy(channelCU.y)
## Deletes the channel to prevent it from further recording data
#del channelY
#del channelU
#del channelCU
#

'''Freq analysis'''

#sw = ri.spawnFile('pr2_arm/gazebo_tune_elbow.xml')
#t = 4
#pos = 7
#fmin=0
#fmax=10
#
#resp=sw.sweep_b(fmin,fmax,t,1)
#times=resp.times
#y=resp.positions
#u=resp.inputs
#cu=u
#
#l=min(len(y),len(u))
#df=1.0/t
#freqs=arange(fmin,fmax,df)
#y=y[:l]
#u=u[:l]
#freqs=freqs[:int(l/2)]
#fu=abs(fft(u)[:int(l/2)])*2/l
#fcu=abs(fft(cu)[:int(l/2)])*2/l
#fy=abs(fft(y)[:int(l/2)])*2/l
#ftr=fy/fu

ri.record()
time.sleep(4)
ri.stop_record()
