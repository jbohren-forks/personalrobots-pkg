#!/usr/bin/env python
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
# Revision $Id: gossipbot.py 1013 2008-05-21 01:08:56Z sfkwc $

## Integration test for peer_subscribe_notify

PKG = 'namelookup'
NAME = 'namelookup_server_test'

import rostools
rostools.update_path(PKG)

import sys, time
import unittest
import random

import rospy, rostest
from namelookup.srv import *

class TestNameLookup(unittest.TestCase):

    
    def test_namelookup_server(self):
        self.names = {}
        self.previously_tried_names = {}
        rospy.wait_for_service('/nameToNumber')
        s = rospy.ServiceProxy('/nameToNumber', NameToNumber)

        ## \todo change this to a random string generator
        alphabet = 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ/1234567890'
        min_length = 1
        max_length = 50
        num_tests = 1000
        tests = []
        for count in xrange(1,num_tests):
            astring = ''
            for x in random.sample(alphabet, random.randint(min_length, max_length)):
                astring += x
            tests.append(astring)
##            print tests
            

        
        ##tests = ['asdf', 'fdsa','asdf']
        

        for x in tests:
            print "Requesting %s"%(x)
            resp = s.call(NameToNumberRequest(x))
            if x in self.names:
                ## assert that we have not seen a different value before
                self.assertEquals(resp.number,self.names[x], "two different values returned for the same string, previous value was %s vs. %d"%( self.names[x], resp.number))
            else:
                ## record this value
                self.names[x] = resp.number
                ##  assert that the name was not previously tested
                self.assertEquals(x in self.previously_tried_names, False )
                self.previously_tried_names[x] = resp.number
                
            print "%s lookus up to %d"%(x, resp.number)            
        
if __name__ == '__main__':
    rostest.run(PKG, NAME, TestNameLookup, sys.argv)
