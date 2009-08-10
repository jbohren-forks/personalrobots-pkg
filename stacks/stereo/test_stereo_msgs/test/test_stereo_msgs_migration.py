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

PKG='test_stereo_msgs'

import roslib
roslib.load_manifest(PKG)

import sys
import struct

import unittest

import rostest
import rosrecord
import rosbagmigration

import re
from cStringIO import StringIO
import os

import rospy

import math

def repack(x):
  return struct.unpack('<f',struct.pack('<f',x))[0]

class TestStereoMsgsMigration(unittest.TestCase):

########### RawStereo ###############

  def get_old_raw_stereo(self):
    raw_stereo_classes = self.load_saved_classes('RawStereo.saved')

    raw_stereo = raw_stereo_classes['image_msgs/RawStereo']
    stereo_info = raw_stereo_classes['image_msgs/StereoInfo']
    cam_info = raw_stereo_classes['image_msgs/CamInfo']
    disparity_info = raw_stereo_classes['image_msgs/DisparityInfo']

    image = raw_stereo_classes['image_msgs/Image']

    multi_array_layout = raw_stereo_classes['std_msgs/MultiArrayLayout']
    multi_array_dimension = raw_stereo_classes['std_msgs/MultiArrayDimension']

    uint8_multi_array = raw_stereo_classes['std_msgs/UInt8MultiArray']
    int8_multi_array = raw_stereo_classes['std_msgs/Int8MultiArray']
    uint16_multi_array = raw_stereo_classes['std_msgs/UInt16MultiArray']
    int16_multi_array = raw_stereo_classes['std_msgs/Int16MultiArray']
    uint32_multi_array = raw_stereo_classes['std_msgs/UInt32MultiArray']
    int32_multi_array = raw_stereo_classes['std_msgs/Int32MultiArray']
    uint64_multi_array = raw_stereo_classes['std_msgs/UInt64MultiArray']
    int64_multi_array = raw_stereo_classes['std_msgs/Int64MultiArray']

    float32_multi_array = raw_stereo_classes['std_msgs/Float32MultiArray']
    float64_multi_array = raw_stereo_classes['std_msgs/Float64MultiArray']
    
    import random
    r = random.Random(5678)

    left_img = image(None,
                     'left_image',
                     'mono',
                     'uint8',
                     uint8_multi_array(multi_array_layout([multi_array_dimension('height', 480, 307200),
                                                    multi_array_dimension('width', 640, 640),
                                                    multi_array_dimension('channel', 1, 1)
                                                    ], 0),
                                [r.randint(0,255) for x in xrange(0,307200)]),
                     int8_multi_array(),
                     uint16_multi_array(),
                     int16_multi_array(),
                     uint32_multi_array(),
                     int32_multi_array(),
                     uint64_multi_array(),
                     int64_multi_array(),
                     float32_multi_array(),
                     float64_multi_array())

    right_img = image(None,
                      'right_image',
                     'mono',
                     'uint8',
                     uint8_multi_array(multi_array_layout([multi_array_dimension('height', 480, 307200),
                                                    multi_array_dimension('width', 640, 640),
                                                    multi_array_dimension('channel', 1, 1)
                                                    ], 0),
                                [r.randint(0,255) for x in xrange(0,307200)]),
                     int8_multi_array(),
                     uint16_multi_array(),
                     int16_multi_array(),
                     uint32_multi_array(),
                     int32_multi_array(),
                     uint64_multi_array(),
                     int64_multi_array(),
                     float32_multi_array(),
                     float64_multi_array())

    return raw_stereo(None,
                      stereo_info(None,
                                  480, 640,
                                  (-0.088779999999999998, -0.00017000000000000001, 0.0015399999999999999),
                                  (-0.00296, 0.01155, 0.00064999999999999997),
                                  (1.0, 0.0, 0.0, -309.70123000000001,
                                   0.0, 1.0, 0.0, -240.53899000000001,
                                   0.0, 0.0, 0.0, 722.28197999999998,
                                   0.0, 0.0, 11.262630896461046, -0.0)),
                      cam_info(None,
                               480, 640,
                               (-0.45795000000000002, 0.29532999999999998, 0.0, 0.0, 0.0),
                               (734.37707999999998, 0.0, 343.25992000000002,
                                0.0, 734.37707999999998, 229.65119999999999,
                                0.0, 0.0, 1.0),
                               (0.99997999999999998, 0.0012800000000000001, -0.0057400000000000003,
                                -0.0012700000000000001, 1.0, 0.00148,
                                0.0057400000000000003, -0.00147, 0.99997999999999998),
                               (722.28197999999998, 0.0, 309.70123000000001, 0.0,
                                0.0, 722.28197999999998, 240.53899000000001, 0.0,
                                0.0, 0.0, 1.0, 0.0)),
                      raw_stereo.IMAGE,
                      left_img,
                      cam_info(None,
                               480, 640,
                               (-0.46471000000000001, 0.28405999999999998, 0.0, 0.0, 0.0),
                               (732.43358999999998, 0.0, 330.20074,
                                0.0, 732.43358999999998, 234.71764999999999,
                                0.0, 0.0, 1.0),
                               (0.99985000000000002, 0.0019, -0.017299999999999999,
                                -0.0019300000000000001, 1.0, -0.0014599999999999999,
                                0.017299999999999999, 0.00149, 0.99985000000000002),
                               (722.28197999999998, 0.0, 309.70123000000001, -64.130840000000006,
                                0.0, 722.28197999999998, 240.53899000000001, 0.0, 
                                0.0, 0.0, 1.0, 0.0)),
                      raw_stereo.IMAGE,
                      right_img,
                      0,
                      disparity_info(),
                      image())


  def get_new_raw_stereo(self):
    from stereo_msgs.msg import RawStereo
    from stereo_msgs.msg import StereoInfo
    from stereo_msgs.msg import DisparityInfo
    from sensor_msgs.msg import CameraInfo
    from sensor_msgs.msg import RegionOfInterest
    from sensor_msgs.msg import Image

    import random
    r = random.Random(5678)
    
    left_img = Image(None,
                     480,
                     640,
                     'mono8',
                     0,
                     640,
                     [r.randint(0,255) for x in xrange(0,307200)])

    right_img = Image(None,
                      480,
                      640,
                      'mono8',
                      0,
                      640,
                      [r.randint(0,255) for x in xrange(0,307200)])

    return RawStereo(None,
                     StereoInfo(None,
                                  480, 640,
                                  (-0.088779999999999998, -0.00017000000000000001, 0.0015399999999999999),
                                  (-0.00296, 0.01155, 0.00064999999999999997),
                                  (1.0, 0.0, 0.0, -309.70123000000001,
                                   0.0, 1.0, 0.0, -240.53899000000001,
                                   0.0, 0.0, 0.0, 722.28197999999998,
                                   0.0, 0.0, 11.262630896461046, -0.0)),
                     CameraInfo(None,
                             480, 640,
                             RegionOfInterest(0,0,480,640),
                               (-0.45795000000000002, 0.29532999999999998, 0.0, 0.0, 0.0),
                             (734.37707999999998, 0.0, 343.25992000000002,
                              0.0, 734.37707999999998, 229.65119999999999,
                              0.0, 0.0, 1.0),
                             (0.99997999999999998, 0.0012800000000000001, -0.0057400000000000003,
                              -0.0012700000000000001, 1.0, 0.00148,
                              0.0057400000000000003, -0.00147, 0.99997999999999998),
                             (722.28197999999998, 0.0, 309.70123000000001, 0.0,
                              0.0, 722.28197999999998, 240.53899000000001, 0.0,
                              0.0, 0.0, 1.0, 0.0)),
                     RawStereo.IMAGE,
                     left_img,
                     CameraInfo(None,
                             480, 640,
                             RegionOfInterest(0,0,480,640),
                             (-0.46471000000000001, 0.28405999999999998, 0.0, 0.0, 0.0),
                             (732.43358999999998, 0.0, 330.20074,
                              0.0, 732.43358999999998, 234.71764999999999,
                              0.0, 0.0, 1.0),
                             (0.99985000000000002, 0.0019, -0.017299999999999999,
                              -0.0019300000000000001, 1.0, -0.0014599999999999999,
                              0.017299999999999999, 0.00149, 0.99985000000000002),
                             (722.28197999999998, 0.0, 309.70123000000001, -64.130840000000006,
                              0.0, 722.28197999999998, 240.53899000000001, 0.0, 
                              0.0, 0.0, 1.0, 0.0)),
                     RawStereo.IMAGE,
                     right_img,
                     0,
                     DisparityInfo(),
                     Image())

  def test_raw_stereo(self):
    self.do_test('raw_stereo', self.get_old_raw_stereo, self.get_new_raw_stereo)



########### Helper functions ###########


  def setUp(self):
    self.pkg_dir = roslib.packages.get_pkg_dir(PKG)


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
#    print "old"
#    print roslib.message.strify_message(msgs[0][1])
#    print "new"
#    print roslib.message.strify_message(m)

    # Strifying them helps make the comparison easier until I figure out why the equality operator is failing
    self.assertTrue(roslib.message.strify_message(msgs[0][1]) == roslib.message.strify_message(m))
#    self.assertTrue(msgs[0][1] == m)

    #Cleanup
    os.remove(oldbag)
    os.remove(newbag)





if __name__ == '__main__':
  rostest.unitrun(PKG, 'test_stereo_msgs_migration', TestStereoMsgsMigration, sys.argv)
