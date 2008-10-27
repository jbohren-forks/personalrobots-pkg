# -*- coding: utf-8 -*-
import rostools
rostools.update_path('robot_mechanism_controllers')

from robot_mechanism_controllers.lqr_controller import *
foo=LQRProxy('left_arm_controller')
foo.cmd(['shoulder_pan_left_joint'],[0],[0],[],[])