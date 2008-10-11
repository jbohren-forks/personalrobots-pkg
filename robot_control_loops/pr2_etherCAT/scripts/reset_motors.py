#! /usr/bin/env python

import rostools
rostools.update_path('mechanism_control')
rostools.update_path('std_srvs')
import rospy, sys
import std_srvs.srv
reset = rospy.ServiceProxy("reset_motors", std_srvs.srv.Empty)
reset()

