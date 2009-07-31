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
#

import roslib
roslib.load_manifest('test_common_msgs')

import sys
import struct

import unittest

import rostest
import rosrecord
import rosbagmigration

import re
from cStringIO import StringIO
import os

class TestCommonMsgsMigration(unittest.TestCase):


########### DiagnosticStatus ###############

  def get_old_diagnostic_status(self):
    diagnostic_status_classes = self.load_saved_classes('DiagnosticStatus.saved')
    
    diagnostic_status = diagnostic_status_classes['diagnostic_msgs/DiagnosticStatus']
    diagnostic_value  = diagnostic_status_classes['diagnostic_msgs/DiagnosticValue']
    diagnostic_string = diagnostic_status_classes['diagnostic_msgs/DiagnosticString']

    return diagnostic_status(0, "abcdef", "ghijkl", [diagnostic_value(42.42, 'foo')], [diagnostic_string('xxxxx', 'bar')])

  def get_new_diagnostic_status(self):
    from diagnostic_msgs.msg import DiagnosticStatus
    from diagnostic_msgs.msg import DiagnosticValue
    from diagnostic_msgs.msg import DiagnosticString

    return DiagnosticStatus(0, "abcdef", "ghijkl", "NONE", [DiagnosticValue(42.42, 'foo')], [DiagnosticString('xxxxx', 'bar')])


  def test_diagnostic_status(self):
    self.do_test('diagnostic_status', self.get_old_diagnostic_status, self.get_new_diagnostic_status)


########### Vector3 ###############


  def get_old_vector3(self):
    vector3_classes = self.load_saved_classes('Vector3.saved')
    
    vector3  = vector3_classes['robot_msgs/Vector3']
    
    return vector3(1.23, 4.56, 7.89)

  def get_new_vector3(self):
    from geometry_msgs.msg import Vector3
    
    return Vector3(1.23, 4.56, 7.89)


  def test_vector3(self):
    self.do_test('vector3', self.get_old_vector3, self.get_new_vector3)


########### Vector3Stamped ###############


  def get_old_vector3_stamped(self):
    vector3_classes = self.load_saved_classes('Vector3.saved')
    vector3_stamped_classes = self.load_saved_classes('Vector3Stamped.saved')
    
    vector3  = vector3_classes['robot_msgs/Vector3']
    vector3_stamped  = vector3_stamped_classes['robot_msgs/Vector3Stamped']
    
    return vector3_stamped(None, vector3(1.23, 4.56, 7.89))

  def get_new_vector3_stamped(self):
    from geometry_msgs.msg import Vector3
    from geometry_msgs.msg import Vector3Stamped
    
    return Vector3Stamped(None, Vector3(1.23, 4.56, 7.89))


  def test_vector3_stamped(self):
    self.do_test('vector3_stamped', self.get_old_vector3_stamped, self.get_new_vector3_stamped)


########### Quaternion ###############


  def get_old_quaternion(self):
    quaternion_classes = self.load_saved_classes('Quaternion.saved')
    
    quaternion  = quaternion_classes['robot_msgs/Quaternion']
    
    return quaternion(1.23, 4.56, 7.89, 1.23)

  def get_new_quaternion(self):
    from geometry_msgs.msg import Quaternion
    
    return Quaternion(1.23, 4.56, 7.89, 1.23)


  def test_quaternion(self):
    self.do_test('quaternion', self.get_old_quaternion, self.get_new_quaternion)


########### QuaternionStamped ###############


  def get_old_quaternion_stamped(self):
    quaternion_classes = self.load_saved_classes('QuaternionStamped.saved')
    
    quaternion_stamped  = quaternion_classes['robot_msgs/QuaternionStamped']
    quaternion  = quaternion_classes['robot_msgs/Quaternion']
    
    return quaternion_stamped(None, quaternion(1.23, 4.56, 7.89, 1.23))

  def get_new_quaternion_stamped(self):
    from geometry_msgs.msg import QuaternionStamped
    from geometry_msgs.msg import Quaternion
    
    return QuaternionStamped(None, Quaternion(1.23, 4.56, 7.89, 1.23))


  def test_quaternion(self):
    self.do_test('quaternion_stamped', self.get_old_quaternion_stamped, self.get_new_quaternion_stamped)


########### Twist ###############


  def get_old_twist(self):
    twist_classes = self.load_saved_classes('Twist.saved')
    
    twist    = twist_classes['robot_msgs/Twist']
    vector3  = twist_classes['robot_msgs/Vector3']
    
    return twist(None, vector3(1,2,3), vector3(4, 5, 6))

  def get_new_twist(self):
    from geometry_msgs.msg import Twist
    from geometry_msgs.msg import Vector3

    return Twist(Vector3(1,2,3), Vector3(4, 5, 6))


  def test_twist(self):
    self.do_test('twist', self.get_old_twist, self.get_new_twist)






########### Helper functions ###########


  def setUp(self):
    self.pkg_dir = roslib.packages.get_pkg_dir("test_common_msgs")


  def load_saved_classes(self,saved_msg):
    f = open("%s/test/%s"%(self.pkg_dir,saved_msg), 'r')

    type_line = f.readline()
    pat = re.compile(r"\[(.*)]:")
    type_match = pat.match(type_line)

    self.assertTrue(type_match is not None, "Full definition file malformed.  First line should be: '[my_package/my_msg]:'")

    saved_type = type_match.groups()[0]
    saved_full_text = f.read()

    saved_classes = roslib.genpy.generate_dynamic(saved_type,saved_full_text)

    self.assertTrue(saved_classes is not None, "Could not generate class from full definition file.")
    self.assertTrue(saved_classes.has_key(saved_type), "Could not generate class from full definition file.")

    return saved_classes

  def do_test(self, name, old_msg, new_msg):
    # Name the bags
    oldbag = "%s/test/%s_old.bag"%(self.pkg_dir,name)
    newbag = "%s/test/%s_new.bag"%(self.pkg_dir,name)

    # Create an old message
    bag = rosrecord.Rebagger(oldbag)
    bag.add("topic", old_msg(), roslib.rostime.Time())
    bag.close()

    # Check and migrate
    res = rosbagmigration.checkbag(oldbag, [])
    self.assertTrue(res is None or res == [], 'Bag not ready to be migrated')
    res = rosbagmigration.fixbag(oldbag, newbag, [])
    self.assertTrue(res, 'Bag not converted successfully')

    # Pull the first message out of the bag
    msgs = [msg for msg in rosrecord.logplayer(newbag)]

    # Reserialize the new message so that floats get screwed up, etc.
    m = new_msg()
    buff = StringIO()
    m.serialize(buff)
    m.deserialize(buff.getvalue())
    
    #Compare

    # Strifying them helps make the comparison easier until I figure out why the equality operator is failing
    self.assertTrue(roslib.message.strify_message(msgs[0][1]) == roslib.message.strify_message(m))
#    self.assertTrue(msgs[0][1] == m)

    #Cleanup
    os.remove(oldbag)
    os.remove(newbag)





if __name__ == '__main__':
  rostest.unitrun('test_common_msgs', 'test_common_msgs_migration', TestCommonMsgsMigration, sys.argv)
