# Copyright (c) 2008, Willow Garage, Inc.
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


## Author Tully Foote tfoote@willowgarage.com

PKG = 'tf'
import roslib
roslib.load_manifest(PKG)

import sys, os

import rospy
import pytf_swig
import transformations
import numpy

from tf.msg import tfMessage

class TransformListener:
    def __init__(self):
        print "Transform Listener initing"
        self.transformer = pytf_swig.pyTransformer()
        #assuming rospy is inited
        rospy.Subscriber("/tf_message", tfMessage, self.callback)


    def callback(self, data):
        for transform in data.transforms:
            print "Got data:", transform.header.frame_id
            rot = transform.transform.rotation
            tr = transform.transform.translation
            ptf = pytf_swig.pyTransform()
            ptf.qx = rot.x
            ptf.qy = rot.y
            ptf.qz = rot.z
            ptf.qw = rot.w
            ptf.x  = tr.x
            ptf.y  = tr.y
            ptf.z  = tr.z
            ptf.frame_id = transform.header.frame_id
            ptf.parent_id = transform.parent_id
            ptf.stamp = transform.header.stamp.to_seconds()

            self.transformer.setTransform(ptf)


    def get_transform(self, frame_id, parent_id, time):
        ptf = self.transformer.getTransform(frame_id, parent_id, time.to_seconds())
        quat = numpy.array([ptf.qx, ptf.qy, ptf.qz, ptf.qw],dtype=numpy.float64)
        rot_mat = transformations.rotation_matrix_from_quaternion(quat)
        print "Rotation: ", rot_mat
        trans = numpy.array([ptf.x, ptf.y, ptf.z])
        trans_mat = transformations.translation_matrix(trans)
        print "Translation: ", trans_mat
        net_tr = transformations.concatenate_transforms(trans_mat, rot_mat)
        print "Net:",  net_tr
        return net_tr

    def get_transform_full(self, target_frame, target_time, source_frame, source_time, fixed_frame):
        ptf = self.transformer.getTransform(target_frame, target_time.to_seconds(), source_frame, source_time.to_seconds(), fixed_frame)
        quat = numpy.array([ptf.qx, ptf.qy, ptf.qz, ptf.qw],dtype=numpy.float64)
        rot_mat = transformations.rotation_matrix_from_quaternion(quat)
        print "Rotation: ", rot_mat
        trans = numpy.array([ptf.x, ptf.y, ptf.z])
        trans_mat = transformations.translation_matrix(trans)
        print "Translation: ", trans_mat
        net_tr = transformations.concatenate_transforms(trans_mat, rot_mat)
        print "Net:",  net_tr
        return net_tr

    

    def can_transform(self, target_frame, source_frame, time):
        return self.transformer.canTransform(target_frame, source_frame, time.to_seconds())

    def can_transform_full(self, target_frame, target_time, source_frame, source_time, fixed_frame):
        return self.transformer.canTransform(target_frame, target_time.to_seconds(), source_frame, source_time.to_seconds(), fixed_frame)

    def get_latest_common_time(self, source_frame, target_frame):
        return self.transformer.getLatestCommonTime(source_frame, target_frame)

    def get_chain_as_string(self, target_frame, target_time, source_frame, source_time, fixed_frame):
        return self.transformer.chainAsString(target_frame, target_time, source_frame, source_time, fixed_frame)

    def all_frames_as_string(self):
        return self.transformer.allFramesAsString()

    def all_frames_as_dot(self):
        return self.transformer.allFramesAsDot()

    def set_extrapolation_limit(self, limit):
        self.transformer.setExtrapolationLimit(limit.to_seconds())


