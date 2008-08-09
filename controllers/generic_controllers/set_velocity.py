#!/usr/bin/env python

import rostools
rostools.update_path('generic_controllers')
import rospy, sys
from generic_controllers.srv import *

s = rospy.ServiceProxy('set_velocity', SetVelocity)
vel = float(sys.argv[1])
resp = s.call(SetVelocityRequest(vel))
print resp.velocity

