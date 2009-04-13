# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest("tf")

import rospy
import tf
import time
import bullet
try:

    rospy.init_node("test_node")
    tfl = tf.TransformListener()
    time.sleep(1)

    # View all frames
    print "All frames are:\n", tfl.all_frames_as_string()

    # dot based introspection
    print "All frame graph is:\n", tfl.all_frames_as_dot()

    
    # test transforming pose
    po = tf.PoseStamped()
    po.frame_id = "base_link"
    po2 = tfl.transform_pose("/map", po)

    print po2.pose

    # test transforming point
    po = tf.PointStamped()
    po.frame_id = "base_link"
    po2 = tfl.transform_point("/map", po)

    # test transforming vector
    po = tf.VectorStamped()
    po.frame_id = "base_link"
    po2 = tfl.transform_vector("/map", po)

    # test transforming quaternion
    po = tf.QuaternionStamped()
    po.frame_id = "base_link"
    po2 = tfl.transform_quaternion("/map", po)

    tr = tf.TransformStamped()

    lps = tf.PoseStamped()
    print "getting stamp"
    print lps.stamp
#    mytime = rospy.Time().now()
    mytime = rospy.Time(10,20)
    lps.stamp = mytime
    print mytime
    print "getting stamp"
    output = lps.stamp
    print output
    print lps.pose
    lps.pose.setIdentity()
    print lps.pose    
    print 'private varilables ', lps._sec, lps._nsec

    transform_stamped = tf.TransformStamped()
    print "getting stamp"
    print transform_stamped.stamp
#    mytime = rospy.Time().now()
    mytime = rospy.Time(10,20)
    transform_stamped.stamp = mytime
    print mytime
    print "getting stamp"
    output = transform_stamped.stamp
    print output
    print transform_stamped.transform
    transform_stamped.transform.setIdentity()
    print transform_stamped.transform
    #    transform_stamped.transform.basis.setEulerZYX(0,0,0)
    print "setting rotation"
    transform_stamped.transform.setRotation(bullet.Quaternion(1,0,0))
    #transform_stamped.transform.setIdentity()
    print transform_stamped.transform
    print 'private varilables ', transform_stamped._sec, transform_stamped._nsec


    pointstamped = tf.PointStamped()
    print "getting stamp"
    print pointstamped.stamp
#    mytime = rospy.Time().now()
    mytime = rospy.Time(10,20)
    pointstamped.stamp = mytime
    print mytime
    print "getting stamp"
    output = pointstamped.stamp
    print output
    print pointstamped.point
    print transform_stamped.transform * pointstamped.point
    print 'private varilables ', pointstamped._sec, pointstamped._nsec


except ValueError, e:
    print "Exception %s Improperly thrown: %s"%(type(e), e)


