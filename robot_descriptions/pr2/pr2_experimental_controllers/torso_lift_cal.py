#!/usr/bin/python                                                               
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

import roslib
import copy
import threading
import sys, os
from time import sleep

# Loads interface with the robot.                                               
roslib.load_manifest('mechanism_control')
roslib.load_manifest('mechanism_bringup')
import rospy
from std_msgs.msg import *
from robot_srvs.srv import *


from mechanism_control import mechanism

spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)

def xml_for():
    return '''
 <controller type="JointLimitCalibrationControllerNode"
    name="cal_torso" topic="cal_torso">
   <calibrate joint="torso_lift_joint"
              actuator="torso_lift_motor"
              transmission="torso_lift_trans"
              velocity="2.0" />
   <pid p="20000" i="25" d="0" iClamp="1000" />
 </controller>'''

def main():
    print 'waiting'
    rospy.wait_for_service('spawn_controller')
    rospy.init_node('cal_torso', anonymous=True)

    try:
        print 'Spawning controller'    
        resp = spawn_controller(xml_for())
        sleep(1)
        print 'Spawned'
        print resp.name[0]
        print ord(resp.ok[0])
        print resp.ok
        rospy.spin()
    except Exception, e:
        print e
        e
    finally:
        mechanism.kill_controller('cal_torso')
        
if __name__ == "__main__":
    main()
