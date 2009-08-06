#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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

PKG = 'functional_m3n_ros'
import roslib; roslib.load_manifest(PKG)

import rospy

import unittest

from functional_m3n_ros.srv import *
from bagserver.srv import *

class FullTestLearningBasic(unittest.TestCase):


    def testLearningSimple(self):
        rospy.sleep(5);
        rospy.wait_for_service('/learn')

        learn_proxy = rospy.ServiceProxy('learn', Learn)
        model_name="test_model_%s" % str(rospy.get_rostime());
        result=learn_proxy(model_name);

        print model_name
        print result
        predictor_proxy = rospy.ServiceProxy('SetModel', SetModel)
        predictor_performance = rospy.ServiceProxy('Performance', QueryPerformanceStats)

        set_model_resul=predictor_proxy(result.model_type,
                                        model_name,
                                        result.model_reference);

        play_history = rospy.ServiceProxy('hist', History)

        begin=rospy.Time(1247098041,895116000); 
        end  =rospy.Time(1247098087,908848000);
        play_history(begin,end,"ALL")
        rospy.sleep(10);

        perf1=predictor_performance();

        rospy.loginfo("Accuracy %f (%f of %f )" %(perf1.accuracy,perf1.correct_weight,perf1.checked_weight))

        self.failIf(perf1.accuracy<0.8);

        set_model_resul=predictor_proxy(result.model_type,
                                        model_name,
                                        result.model_reference);

        begin=rospy.Time( 1247098100, 316937000);
        end  =rospy.Time( 1247098133, 726237000);

        play_history(begin,end,"ALL")

        rospy.sleep(10);
        perf2=predictor_performance();
        rospy.loginfo("Accuracy %f (%f of %f )" %(perf2.accuracy,perf2.correct_weight,perf2.checked_weight))

        self.failIf(perf2.accuracy<0.7);

from threading import Thread

if __name__ == "__main__":
    import rostest
    rospy.init_node("test_content");

    rostest.rosrun('functional_m3n_ros','test_full_training_1',FullTestLearningBasic)

